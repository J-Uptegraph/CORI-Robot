#!/usr/bin/env python3
"""
DevMetrics Pro - Terminal Developer Analytics Platform
Production-grade metrics visualization in ASCII terminal format
Version: 2.0.0 - Terminal Impact Edition
"""

import subprocess
import datetime
import argparse
import re
import os
import json
import sqlite3
from pathlib import Path
from collections import Counter, defaultdict
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Tuple, Any
import logging
import math
import calendar

# --- Configuration & Constants ---
@dataclass
class MetricsConfig:
    """Production configuration for DevMetrics Pro"""
    data_dir: str = ".devmetrics"
    db_file: str = "devmetrics.db"
    cache_duration: int = 3600
    max_commits: int = 10000
    terminal_width: int = 120
    chart_height: int = 15

# --- Data Models ---
@dataclass
class ProductivityMetrics:
    commits_per_day: float
    lines_per_commit: float
    code_churn_ratio: float
    velocity_trend: str
    consistency_score: float
    peak_productivity_hour: int
    active_days_percentage: float
    commit_frequency_score: float

@dataclass
class TechnicalMetrics:
    languages_distribution: Dict[str, float]
    complexity_trend: List[float]
    refactoring_ratio: float
    testing_coverage: float
    documentation_ratio: float
    architecture_score: float
    code_quality_index: float

@dataclass
class ImpactMetrics:
    total_lines_of_code: int
    files_impacted: int
    repository_coverage: float
    collaboration_index: float
    innovation_score: float
    technical_debt_trend: str
    delivery_velocity: float

# --- ASCII Art & Visualization Functions ---
class TerminalVisualizer:
    """Advanced terminal visualization with ASCII charts and heatmaps"""
    
    def __init__(self, width=120):
        self.width = width
        self.chart_chars = {
            'full': '█',
            'seven_eighths': '▉',
            'three_quarters': '▊',
            'five_eighths': '▋', 
            'half': '▌',
            'three_eighths': '▍',
            'quarter': '▎',
            'eighth': '▏',
            'empty': ' '
        }
    
    def create_bar_chart(self, data: Dict[str, float], title: str, max_width: int = 60) -> str:
        """Create horizontal bar chart with precise scaling"""
        if not data:
            return f"{title}\n{'='*len(title)}\nNo data available\n"
        
        max_value = max(data.values())
        if max_value == 0:
            max_value = 1
        
        lines = [title, '=' * len(title)]
        
        for label, value in sorted(data.items(), key=lambda x: x[1], reverse=True):
            # Calculate bar length
            bar_length = int((value / max_value) * max_width)
            remainder = ((value / max_value) * max_width) - bar_length
            
            # Create bar with sub-character precision
            bar = self.chart_chars['full'] * bar_length
            if remainder > 0.875:
                bar += self.chart_chars['seven_eighths']
            elif remainder > 0.75:
                bar += self.chart_chars['three_quarters']
            elif remainder > 0.625:
                bar += self.chart_chars['five_eighths']
            elif remainder > 0.5:
                bar += self.chart_chars['half']
            elif remainder > 0.375:
                bar += self.chart_chars['three_eighths']
            elif remainder > 0.25:
                bar += self.chart_chars['quarter']
            elif remainder > 0.125:
                bar += self.chart_chars['eighth']
            
            # Format line with value
            value_str = f"{value:.1f}" if isinstance(value, float) else str(value)
            lines.append(f"{label:<20} |{bar:<{max_width}} {value_str}")
        
        return '\n'.join(lines) + '\n'
    
    def create_line_chart(self, data: List[float], title: str, height: int = 15, width: int = 80) -> str:
        """Create ASCII line chart with trend visualization"""
        if not data or len(data) < 2:
            return f"{title}\nInsufficient data for trend analysis\n"
        
        # Normalize data to chart height
        min_val, max_val = min(data), max(data)
        if max_val == min_val:
            max_val = min_val + 1
        
        normalized = []
        for val in data:
            norm = int(((val - min_val) / (max_val - min_val)) * (height - 1))
            normalized.append(norm)
        
        # Create chart grid
        chart_lines = []
        chart_lines.append(title)
        chart_lines.append('=' * len(title))
        chart_lines.append(f"Max: {max_val:.2f}")
        
        # Draw chart from top to bottom
        for y in range(height - 1, -1, -1):
            line = ""
            for x, norm_val in enumerate(normalized):
                if x < len(normalized) - 1:
                    next_val = normalized[x + 1]
                    # Draw line between points
                    if norm_val == y:
                        if next_val > norm_val:
                            line += "/"
                        elif next_val < norm_val:
                            line += "\\"
                        else:
                            line += "─"
                    elif y == next_val and abs(next_val - norm_val) == 1:
                        line += "/" if next_val > norm_val else "\\"
                    elif min(norm_val, next_val) < y < max(norm_val, next_val):
                        line += "|"
                    else:
                        line += " "
                else:
                    line += "*" if norm_val == y else " "
            
            # Add y-axis labels
            y_val = min_val + (y / (height - 1)) * (max_val - min_val)
            chart_lines.append(f"{y_val:6.1f} |{line}")
        
        chart_lines.append(f"Min: {min_val:.2f}")
        chart_lines.append("")
        
        return '\n'.join(chart_lines)
    
    def create_heatmap(self, data: Dict[str, Dict[str, float]], title: str) -> str:
        """Create ASCII heatmap for time-based data"""
        if not data:
            return f"{title}\nNo data available\n"
        
        lines = [title, '=' * len(title)]
        
        # Get all time periods and sort
        all_periods = set()
        for day_data in data.values():
            all_periods.update(day_data.keys())
        periods = sorted(all_periods)
        
        # Create header
        header = "Day      " + "".join(f"{p:>4}" for p in periods)
        lines.append(header)
        lines.append("-" * len(header))
        
        # Create intensity scale
        all_values = []
        for day_data in data.values():
            all_values.extend(day_data.values())
        
        if all_values:
            max_intensity = max(all_values)
            if max_intensity > 0:
                for day, day_data in sorted(data.items()):
                    line = f"{day:<8} "
                    for period in periods:
                        value = day_data.get(period, 0)
                        intensity = value / max_intensity
                        
                        if intensity >= 0.8:
                            char = '█'
                        elif intensity >= 0.6:
                            char = '▓'
                        elif intensity >= 0.4:
                            char = '▒'
                        elif intensity >= 0.2:
                            char = '░'
                        elif intensity > 0:
                            char = '·'
                        else:
                            char = ' '
                        
                        line += f" {char:>2} "
                    lines.append(line)
        
        # Add legend
        lines.append("")
        lines.append("Intensity: █ High  ▓ Med-High  ▒ Medium  ░ Low  · Minimal   Empty")
        lines.append("")
        
        return '\n'.join(lines)
    
    def create_distribution_chart(self, data: List[float], title: str, bins: int = 10) -> str:
        """Create histogram/distribution chart"""
        if not data:
            return f"{title}\nNo data available\n"
        
        # Calculate histogram
        min_val, max_val = min(data), max(data)
        if max_val == min_val:
            return f"{title}\nAll values are identical: {min_val}\n"
        
        bin_width = (max_val - min_val) / bins
        histogram = [0] * bins
        
        for value in data:
            bin_idx = min(int((value - min_val) / bin_width), bins - 1)
            histogram[bin_idx] += 1
        
        max_count = max(histogram)
        chart_width = 50
        
        lines = [title, '=' * len(title)]
        
        for i, count in enumerate(histogram):
            bin_start = min_val + i * bin_width
            bin_end = bin_start + bin_width
            bar_length = int((count / max_count) * chart_width) if max_count > 0 else 0
            bar = '█' * bar_length
            
            lines.append(f"{bin_start:6.1f}-{bin_end:6.1f} |{bar:<{chart_width}} {count}")
        
        lines.append("")
        return '\n'.join(lines)

# --- Core Analytics Engine ---
class DevMetricsAnalyzer:
    """Production analytics engine with terminal visualization"""
    
    def __init__(self, config: MetricsConfig):
        self.config = config
        self.viz = TerminalVisualizer(config.terminal_width)
        self.db_path = Path(config.data_dir) / config.db_file
        self.setup_database()
        self.setup_logging()
        
    def setup_logging(self):
        """Configure logging"""
        Path(self.config.data_dir).mkdir(exist_ok=True)
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(Path(self.config.data_dir) / 'devmetrics.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
        
    def setup_database(self):
        """Initialize metrics database"""
        Path(self.config.data_dir).mkdir(exist_ok=True)
        
        with sqlite3.connect(self.db_path) as conn:
            conn.executescript("""
                CREATE TABLE IF NOT EXISTS commits (
                    hash TEXT PRIMARY KEY,
                    timestamp DATETIME,
                    message TEXT,
                    author TEXT,
                    files_changed INTEGER,
                    lines_added INTEGER,
                    lines_deleted INTEGER,
                    complexity_score REAL,
                    primary_language TEXT,
                    semantic_category TEXT
                );
                
                CREATE TABLE IF NOT EXISTS metrics_cache (
                    date DATE PRIMARY KEY,
                    productivity_score REAL,
                    quality_score REAL,
                    impact_score REAL
                );
                
                CREATE INDEX IF NOT EXISTS idx_commits_timestamp ON commits(timestamp);
            """)
    
    def collect_git_data(self, since: str = "1y"):
        """Collect comprehensive git repository data"""
        self.logger.info(f"Analyzing repository data since {since}")
        
        since_date = self._parse_since_date(since)
        
        print("DEVMETRICS PRO - REPOSITORY ANALYSIS")
        print("=" * 50)
        print("Status: Collecting commit data...")
        
        commits_data = self._fetch_git_log(since_date)
        
        print(f"Status: Processing {len(commits_data)} commits...")
        commits_data = self._enhance_commit_data(commits_data)
        
        print("Status: Computing analytics...")
        return commits_data
    
    def _fetch_git_log(self, since_date: str):
        """Fetch git log with comprehensive statistics"""
        try:
            cmd = [
                "git", "log", f"--since={since_date}",
                "--pretty=format:%H|%ai|%an|%s",
                "--numstat", "--no-merges"
            ]
            
            output = subprocess.check_output(cmd, text=True, cwd=".")
            commits = []
            current_commit = None
            
            for line in output.splitlines():
                if '|' in line and line.count('|') == 3:
                    if current_commit:
                        commits.append(current_commit)
                    
                    hash_, timestamp, author, message = line.split('|', 3)
                    current_commit = {
                        'hash': hash_,
                        'timestamp': self._parse_timestamp(timestamp),
                        'author': author,
                        'message': message.strip(),
                        'files_changed': 0,
                        'lines_added': 0,
                        'lines_deleted': 0,
                        'modified_files': []
                    }
                elif '\t' in line and current_commit:
                    try:
                        parts = line.split('\t')
                        if len(parts) >= 3:
                            added, deleted, filename = parts[0], parts[1], parts[2]
                            if added.isdigit() and deleted.isdigit():
                                current_commit['lines_added'] += int(added)
                                current_commit['lines_deleted'] += int(deleted)
                                current_commit['files_changed'] += 1
                                current_commit['modified_files'].append(filename)
                    except (ValueError, IndexError):
                        continue
            
            if current_commit:
                commits.append(current_commit)
            
            return commits
            
        except subprocess.CalledProcessError as e:
            self.logger.error(f"Git analysis failed: {e}")
            return []
    
    def _parse_timestamp(self, timestamp_str):
        """Parse git timestamp to datetime object"""
        try:
            if timestamp_str.endswith('Z'):
                timestamp_str = timestamp_str[:-1] + '+00:00'
            elif '+' not in timestamp_str and timestamp_str.count('-') <= 2:
                timestamp_str += '+00:00'
            return datetime.datetime.fromisoformat(timestamp_str)
        except:
            return datetime.datetime.now()
    
    def _enhance_commit_data(self, commits_data):
        """Enhance commits with analytics metadata"""
        if not commits_data:
            return commits_data
        
        # Language detection
        language_map = {
            '.py': 'Python', '.js': 'JavaScript', '.ts': 'TypeScript',
            '.java': 'Java', '.cpp': 'C++', '.c': 'C', '.cs': 'C#',
            '.go': 'Go', '.rs': 'Rust', '.rb': 'Ruby', '.php': 'PHP',
            '.swift': 'Swift', '.kt': 'Kotlin', '.scala': 'Scala',
            '.sh': 'Shell', '.sql': 'SQL', '.html': 'HTML', '.css': 'CSS',
            '.jsx': 'React', '.tsx': 'React', '.vue': 'Vue', '.yml': 'YAML',
            '.json': 'JSON', '.md': 'Markdown'
        }
        
        # Semantic categorization patterns
        semantic_patterns = {
            'feature': r'\b(feat|feature|add|implement|create|new)\b',
            'bugfix': r'\b(fix|bug|patch|resolve|correct|error)\b',
            'refactor': r'\b(refactor|cleanup|reorganize|optimize|improve)\b',
            'docs': r'\b(doc|documentation|readme|comment)\b',
            'test': r'\b(test|spec|unit|integration|coverage)\b',
            'chore': r'\b(chore|update|upgrade|bump|config|build)\b',
            'style': r'\b(style|format|lint|prettier|eslint)\b',
            'performance': r'\b(perf|performance|speed|optimize)\b'
        }
        
        # Calculate normalization factors
        max_lines = max((c.get('lines_added', 0) + c.get('lines_deleted', 0) for c in commits_data), default=1)
        max_files = max((c.get('files_changed', 0) for c in commits_data), default=1)
        
        for commit in commits_data:
            # Language analysis
            files = commit.get('modified_files', [])
            languages = []
            for file_path in files:
                ext = Path(str(file_path)).suffix.lower()
                if ext in language_map:
                    languages.append(language_map[ext])
            
            commit['primary_language'] = Counter(languages).most_common(1)[0][0] if languages else 'Unknown'
            commit['languages_count'] = len(set(languages))
            
            # Semantic categorization
            message = commit.get('message', '').lower()
            category = 'other'
            for cat, pattern in semantic_patterns.items():
                if re.search(pattern, message):
                    category = cat
                    break
            commit['semantic_category'] = category
            
            # Complexity scoring
            lines_changed = commit.get('lines_added', 0) + commit.get('lines_deleted', 0)
            files_changed = commit.get('files_changed', 0)
            
            complexity_score = (
                (lines_changed / max_lines) * 0.5 +
                (files_changed / max_files) * 0.3 +
                (commit['languages_count'] / 5) * 0.2
            ) * 100
            
            commit['complexity_score'] = min(complexity_score, 100)
            commit['impact_score'] = complexity_score * (1 + len(languages) * 0.1)
        
        return commits_data
    
    def _parse_since_date(self, since: str) -> str:
        """Parse time period specification"""
        match = re.match(r'(\d+)([yYmMwWdD])', since)
        if not match:
            return "1 year ago"
        
        value, unit = int(match[1]), match[2].lower()
        units_map = {'y': 'year', 'm': 'month', 'w': 'week', 'd': 'day'}
        unit_name = units_map.get(unit, 'day')
        
        return f"{value} {unit_name}{'s' if value > 1 else ''} ago"
    
    def calculate_productivity_metrics(self, commits_data) -> ProductivityMetrics:
        """Calculate comprehensive productivity analytics"""
        if not commits_data:
            return ProductivityMetrics(0, 0, 0, "stable", 0, 0, 0, 0)
        
        # Time-based analysis
        daily_commits = defaultdict(int)
        hourly_commits = defaultdict(int)
        
        total_lines = sum(c.get('lines_added', 0) + c.get('lines_deleted', 0) for c in commits_data)
        total_commits = len(commits_data)
        
        for commit in commits_data:
            timestamp = commit.get('timestamp')
            if timestamp and hasattr(timestamp, 'date'):
                daily_commits[timestamp.date()] += 1
                hourly_commits[timestamp.hour] += 1
        
        # Core metrics
        active_days = len(daily_commits)
        commits_per_day = total_commits / max(active_days, 1)
        lines_per_commit = total_lines / max(total_commits, 1)
        
        # Churn analysis
        total_added = sum(c.get('lines_added', 0) for c in commits_data)
        total_deleted = sum(c.get('lines_deleted', 0) for c in commits_data)
        code_churn_ratio = total_deleted / max(total_added, 1)
        
        # Velocity trend analysis
        if active_days >= 14:
            sorted_days = sorted(daily_commits.keys())
            mid_point = len(sorted_days) // 2
            recent_avg = sum(daily_commits[d] for d in sorted_days[mid_point:]) / max(len(sorted_days[mid_point:]), 1)
            older_avg = sum(daily_commits[d] for d in sorted_days[:mid_point]) / max(len(sorted_days[:mid_point]), 1)
            
            if recent_avg > older_avg * 1.15:
                velocity_trend = "accelerating"
            elif recent_avg < older_avg * 0.85:
                velocity_trend = "declining"
            else:
                velocity_trend = "stable"
        else:
            velocity_trend = "stable"
        
        # Consistency scoring
        daily_counts = list(daily_commits.values())
        if daily_counts:
            mean_daily = sum(daily_counts) / len(daily_counts)
            variance = sum((x - mean_daily) ** 2 for x in daily_counts) / len(daily_counts)
            std_dev = math.sqrt(variance)
            consistency_score = max(0, 100 - (std_dev / max(mean_daily, 1)) * 100)
        else:
            consistency_score = 0
        
        # Peak productivity hour
        peak_hour = max(hourly_commits, key=hourly_commits.get) if hourly_commits else 12
        
        # Activity percentage calculation
        start_date = min(commit['timestamp'].date() for commit in commits_data if commit.get('timestamp'))
        end_date = max(commit['timestamp'].date() for commit in commits_data if commit.get('timestamp'))
        total_possible_days = (end_date - start_date).days + 1
        active_days_percentage = (active_days / max(total_possible_days, 1)) * 100
        
        # Frequency scoring (commits per active day)
        commit_frequency_score = min(100, (commits_per_day / 3) * 100)
        
        return ProductivityMetrics(
            commits_per_day=round(commits_per_day, 2),
            lines_per_commit=round(lines_per_commit, 1),
            code_churn_ratio=round(code_churn_ratio, 3),
            velocity_trend=velocity_trend,
            consistency_score=round(consistency_score, 1),
            peak_productivity_hour=peak_hour,
            active_days_percentage=round(active_days_percentage, 1),
            commit_frequency_score=round(commit_frequency_score, 1)
        )
    
    def calculate_technical_metrics(self, commits_data) -> TechnicalMetrics:
        """Calculate technical proficiency analytics"""
        if not commits_data:
            return TechnicalMetrics({}, [], 0, 0, 0, 0, 0)
        
        # Language distribution analysis
        language_stats = defaultdict(lambda: {'commits': 0, 'lines': 0, 'complexity': []})
        
        for commit in commits_data:
            lang = commit.get('primary_language', 'Unknown')
            if lang != 'Unknown':
                language_stats[lang]['commits'] += 1
                language_stats[lang]['lines'] += commit.get('lines_added', 0)
                language_stats[lang]['complexity'].append(commit.get('complexity_score', 0))
        
        # Calculate language distribution as percentages
        total_commits = len(commits_data)
        languages_distribution = {}
        for lang, stats in language_stats.items():
            percentage = (stats['commits'] / total_commits) * 100
            languages_distribution[lang] = round(percentage, 1)
        
        # Complexity trend analysis (by week)
        weekly_complexity = defaultdict(list)
        for commit in commits_data:
            timestamp = commit.get('timestamp')
            if timestamp:
                week_key = timestamp.isocalendar()[:2]  # (year, week)
                weekly_complexity[week_key].append(commit.get('complexity_score', 0))
        
        complexity_trend = []
        for week_key in sorted(weekly_complexity.keys()):
            avg_complexity = sum(weekly_complexity[week_key]) / len(weekly_complexity[week_key])
            complexity_trend.append(avg_complexity)
        
        # Quality metrics
        category_counts = Counter(c.get('semantic_category', 'other') for c in commits_data)
        
        refactor_commits = category_counts.get('refactor', 0)
        refactoring_ratio = (refactor_commits / total_commits) * 100
        
        test_commits = category_counts.get('test', 0)
        testing_coverage = (test_commits / total_commits) * 100
        
        doc_commits = category_counts.get('docs', 0)
        documentation_ratio = (doc_commits / total_commits) * 100
        
        # Architecture score (based on complexity and refactoring)
        avg_complexity = sum(c.get('complexity_score', 0) for c in commits_data) / total_commits
        architecture_score = min(100, avg_complexity + refactoring_ratio)
        
        # Code quality index
        quality_categories = ['feature', 'bugfix', 'refactor', 'test']
        quality_commits = sum(category_counts.get(cat, 0) for cat in quality_categories)
        code_quality_index = (quality_commits / total_commits) * 100
        
        return TechnicalMetrics(
            languages_distribution=languages_distribution,
            complexity_trend=complexity_trend,
            refactoring_ratio=round(refactoring_ratio, 1),
            testing_coverage=round(testing_coverage, 1),
            documentation_ratio=round(documentation_ratio, 1),
            architecture_score=round(architecture_score, 1),
            code_quality_index=round(code_quality_index, 1)
        )
    
    def calculate_impact_metrics(self, commits_data) -> ImpactMetrics:
        """Calculate repository and team impact metrics"""
        if not commits_data:
            return ImpactMetrics(0, 0, 0, 0, 0, "stable", 0)
        
        # Repository impact analysis
        total_lines_of_code = sum(c.get('lines_added', 0) for c in commits_data)
        files_impacted = len(set(f for c in commits_data for f in c.get('modified_files', [])))
        
        # Repository coverage (estimate based on file diversity)
        file_extensions = set()
        for commit in commits_data:
            for file_path in commit.get('modified_files', []):
                ext = Path(str(file_path)).suffix
                if ext:
                    file_extensions.add(ext)
        
        repository_coverage = min(100, len(file_extensions) * 10)  # Rough estimate
        
        # Collaboration analysis (multiple file types indicates cross-functional work)
        collaboration_index = min(100, len(file_extensions) * 8)
        
        # Innovation score (based on new features and complexity)
        feature_commits = sum(1 for c in commits_data if c.get('semantic_category') == 'feature')
        avg_complexity = sum(c.get('complexity_score', 0) for c in commits_data) / len(commits_data)
        innovation_score = min(100, (feature_commits / len(commits_data)) * 100 + avg_complexity * 0.5)
        
        # Technical debt trend
        chore_commits = sum(1 for c in commits_data if c.get('semantic_category') == 'chore')
        refactor_commits = sum(1 for c in commits_data if c.get('semantic_category') == 'refactor')
        debt_reduction_ratio = (chore_commits + refactor_commits) / len(commits_data)
        
        if debt_reduction_ratio > 0.25:
            technical_debt_trend = "improving"
        elif debt_reduction_ratio < 0.10:
            technical_debt_trend = "degrading"
        else:
            technical_debt_trend = "stable"
        
        # Delivery velocity (commits per week)
        timestamps = [c['timestamp'] for c in commits_data if c.get('timestamp')]
        if timestamps:
            time_span = (max(timestamps) - min(timestamps)).days
            weeks = max(time_span / 7, 1)
            delivery_velocity = len(commits_data) / weeks
        else:
            delivery_velocity = 0
        
        return ImpactMetrics(
            total_lines_of_code=total_lines_of_code,
            files_impacted=files_impacted,
            repository_coverage=round(repository_coverage, 1),
            collaboration_index=round(collaboration_index, 1),
            innovation_score=round(innovation_score, 1),
            technical_debt_trend=technical_debt_trend,
            delivery_velocity=round(delivery_velocity, 2)
        )
    
    def generate_comprehensive_report(self, commits_data):
        """Generate full terminal-based analytics report"""
        if not commits_data:
            return "ERROR: No commit data available for analysis"
        
        # Calculate all metrics
        productivity = self.calculate_productivity_metrics(commits_data)
        technical = self.calculate_technical_metrics(commits_data)
        impact = self.calculate_impact_metrics(commits_data)
        
        # Repository info
        repo_name = self._get_repo_name()
        analysis_period = self._get_analysis_period(commits_data)
        
        report_sections = []
        
        # Header
        header = f"""
DEVMETRICS PRO - COMPREHENSIVE DEVELOPER ANALYTICS
{'=' * 80}
Repository: {repo_name}
Analysis Period: {analysis_period}
Total Commits Analyzed: {len(commits_data)}
Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
{'=' * 80}
"""
        report_sections.append(header)
        
        # Executive Summary
        summary = f"""
EXECUTIVE SUMMARY
{'-' * 40}
Productivity Level:     {self._get_productivity_rating(productivity)}
Technical Proficiency:  {self._get_technical_rating(technical)}
Repository Impact:      {self._get_impact_rating(impact)}
Code Quality Index:     {technical.code_quality_index}/100
Delivery Velocity:      {impact.delivery_velocity} commits/week
Consistency Score:      {productivity.consistency_score}/100
"""
        report_sections.append(summary)
        
        # Productivity Analytics
        productivity_chart = self.viz.create_bar_chart({
            'Commits/Day': productivity.commits_per_day,
            'Lines/Commit': productivity.lines_per_commit / 10,  # Scale for visualization
            'Consistency': productivity.consistency_score,
            'Frequency Score': productivity.commit_frequency_score,
            'Activity %': productivity.active_days_percentage
        }, "PRODUCTIVITY METRICS")
        report_sections.append(productivity_chart)
        
        # Technical Distribution
        if technical.languages_distribution:
            lang_chart = self.viz.create_bar_chart(
                technical.languages_distribution, 
                "LANGUAGE DISTRIBUTION (%)"
            )
            report_sections.append(lang_chart)
        
        # Complexity Trend Analysis
        if technical.complexity_trend and len(technical.complexity_trend) > 1:
            complexity_chart = self.viz.create_line_chart(
                technical.complexity_trend,
                "COMPLEXITY TREND OVER TIME",
                height=12,
                width=60
            )
            report_sections.append(complexity_chart)
        
        # Activity Heatmap (Day of Week vs Hour)
        activity_heatmap = self._generate_activity_heatmap(commits_data)
        if activity_heatmap:
            heatmap_viz = self.viz.create_heatmap(activity_heatmap, "ACTIVITY HEATMAP (DAY vs HOUR)")
            report_sections.append(heatmap_viz)
        
        # Commit Size Distribution
        commit_sizes = [c.get('lines_added', 0) + c.get('lines_deleted', 0) for c in commits_data]
        if commit_sizes:
            size_dist = self.viz.create_distribution_chart(
                commit_sizes,
                "COMMIT SIZE DISTRIBUTION",
                bins=8
            )
            report_sections.append(size_dist)
        
        # Quality Metrics Table
        quality_metrics = f"""
CODE QUALITY METRICS
{'-' * 40}
Testing Coverage:       {technical.testing_coverage:.1f}%
Refactoring Ratio:      {technical.refactoring_ratio:.1f}%
Documentation Ratio:    {technical.documentation_ratio:.1f}%
Architecture Score:     {technical.architecture_score:.1f}/100
Code Quality Index:     {technical.code_quality_index:.1f}/100
Technical Debt Trend:   {impact.technical_debt_trend.upper()}
"""
        report_sections.append(quality_metrics)
        
        # Impact Analysis
        impact_chart = self.viz.create_bar_chart({
            'Lines of Code': impact.total_lines_of_code / 100,  # Scale down
            'Files Impacted': impact.files_impacted,
            'Repo Coverage': impact.repository_coverage,
            'Collaboration': impact.collaboration_index,
            'Innovation': impact.innovation_score
        }, "REPOSITORY IMPACT METRICS")
        report_sections.append(impact_chart)
        
        # Semantic Analysis
        semantic_analysis = self._generate_semantic_analysis(commits_data)
        report_sections.append(semantic_analysis)
        
        # Performance Insights
        insights = self._generate_performance_insights(productivity, technical, impact)
        report_sections.append(insights)
        
        # Recommendations
        recommendations = self._generate_recommendations(productivity, technical, impact)
        report_sections.append(recommendations)
        
        return '\n'.join(report_sections)
    
    def _generate_activity_heatmap(self, commits_data) -> Dict[str, Dict[str, float]]:
        """Generate activity heatmap data for day of week vs hour of day"""
        activity_map = {}
        days = ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday']
        
        # Initialize heatmap
        for day in days:
            activity_map[day] = {}
            for hour in range(24):
                activity_map[day][str(hour)] = 0
        
        # Populate with commit data
        for commit in commits_data:
            timestamp = commit.get('timestamp')
            if timestamp and hasattr(timestamp, 'weekday'):
                day_name = days[timestamp.weekday()]
                hour = str(timestamp.hour)
                activity_map[day_name][hour] += 1
        
        return activity_map
    
    def _generate_semantic_analysis(self, commits_data) -> str:
        """Generate semantic commit analysis"""
        category_counts = Counter(c.get('semantic_category', 'other') for c in commits_data)
        total = len(commits_data)
        
        semantic_data = {}
        for category, count in category_counts.items():
            percentage = (count / total) * 100
            semantic_data[category.upper()] = percentage
        
        return self.viz.create_bar_chart(semantic_data, "SEMANTIC COMMIT ANALYSIS (%)")
    
    def _generate_performance_insights(self, productivity: ProductivityMetrics, 
                                     technical: TechnicalMetrics, 
                                     impact: ImpactMetrics) -> str:
        """Generate data-driven performance insights"""
        insights = []
        insights.append("PERFORMANCE INSIGHTS")
        insights.append("-" * 40)
        
        # Productivity insights
        if productivity.commits_per_day > 2.0:
            insights.append("+ HIGH PRODUCTIVITY: Above-average commit frequency detected")
        elif productivity.commits_per_day < 0.5:
            insights.append("- LOW ACTIVITY: Consider increasing development velocity")
        
        if productivity.consistency_score > 80:
            insights.append("+ EXCELLENT CONSISTENCY: Highly regular development patterns")
        elif productivity.consistency_score < 40:
            insights.append("- INCONSISTENT WORKFLOW: Variable development patterns detected")
        
        # Technical insights
        if technical.code_quality_index > 75:
            insights.append("+ HIGH CODE QUALITY: Strong focus on structured development")
        
        if technical.refactoring_ratio > 15:
            insights.append("+ PROACTIVE MAINTENANCE: High refactoring activity indicates quality focus")
        elif technical.refactoring_ratio < 5:
            insights.append("- LIMITED REFACTORING: Consider increasing code maintenance activities")
        
        if technical.testing_coverage > 20:
            insights.append("+ TESTING FOCUS: Good test coverage commitment")
        elif technical.testing_coverage < 5:
            insights.append("- LOW TEST COVERAGE: Recommend increasing automated testing")
        
        # Impact insights
        if impact.innovation_score > 70:
            insights.append("+ HIGH INNOVATION: Strong feature development and complexity handling")
        
        if impact.collaboration_index > 60:
            insights.append("+ CROSS-FUNCTIONAL: Evidence of diverse technical involvement")
        
        # Velocity insights
        velocity_rating = "HIGH" if productivity.velocity_trend == "accelerating" else \
                         "DECLINING" if productivity.velocity_trend == "declining" else "STABLE"
        insights.append(f"VELOCITY TREND: {velocity_rating}")
        
        return '\n'.join(insights) + '\n'
    
    def _generate_recommendations(self, productivity: ProductivityMetrics,
                                technical: TechnicalMetrics,
                                impact: ImpactMetrics) -> str:
        """Generate actionable recommendations"""
        recommendations = []
        recommendations.append("STRATEGIC RECOMMENDATIONS")
        recommendations.append("-" * 40)
        
        # Productivity recommendations
        if productivity.commits_per_day < 1.0:
            recommendations.append("1. INCREASE VELOCITY: Consider smaller, more frequent commits")
        
        if productivity.consistency_score < 60:
            recommendations.append("2. IMPROVE CONSISTENCY: Establish regular development schedule")
        
        # Technical recommendations
        if technical.testing_coverage < 15:
            recommendations.append("3. ENHANCE TESTING: Implement comprehensive test strategy")
        
        if technical.documentation_ratio < 10:
            recommendations.append("4. DOCUMENTATION: Increase inline and project documentation")
        
        if technical.refactoring_ratio < 10:
            recommendations.append("5. CODE MAINTENANCE: Schedule regular refactoring sessions")
        
        # Impact recommendations
        if impact.innovation_score < 50:
            recommendations.append("6. INNOVATION FOCUS: Increase feature development complexity")
        
        if len(technical.languages_distribution) < 2:
            recommendations.append("7. TECHNICAL DIVERSITY: Explore additional technologies")
        
        # Add positive reinforcements
        strengths = []
        if productivity.consistency_score > 80:
            strengths.append("Excellent development consistency")
        if technical.code_quality_index > 70:
            strengths.append("Strong code quality practices")
        if impact.innovation_score > 60:
            strengths.append("High innovation and feature delivery")
        
        if strengths:
            recommendations.append("")
            recommendations.append("IDENTIFIED STRENGTHS:")
            for strength in strengths:
                recommendations.append(f"+ {strength}")
        
        return '\n'.join(recommendations) + '\n'
    
    def _get_productivity_rating(self, productivity: ProductivityMetrics) -> str:
        """Get overall productivity rating"""
        score = (productivity.commits_per_day * 10 + productivity.consistency_score + 
                productivity.commit_frequency_score) / 3
        
        if score >= 80: return "EXCEPTIONAL"
        elif score >= 60: return "HIGH"
        elif score >= 40: return "MODERATE"
        else: return "DEVELOPING"
    
    def _get_technical_rating(self, technical: TechnicalMetrics) -> str:
        """Get technical proficiency rating"""
        score = (technical.code_quality_index + technical.architecture_score + 
                len(technical.languages_distribution) * 10) / 3
        
        if score >= 80: return "EXPERT"
        elif score >= 60: return "PROFICIENT"
        elif score >= 40: return "COMPETENT"
        else: return "DEVELOPING"
    
    def _get_impact_rating(self, impact: ImpactMetrics) -> str:
        """Get repository impact rating"""
        score = (impact.repository_coverage + impact.collaboration_index + 
                impact.innovation_score) / 3
        
        if score >= 80: return "TRANSFORMATIVE"
        elif score >= 60: return "SIGNIFICANT"
        elif score >= 40: return "MODERATE"
        else: return "LIMITED"
    
    def _get_repo_name(self) -> str:
        """Get repository name"""
        try:
            repo_path = subprocess.check_output(["git", "rev-parse", "--show-toplevel"], text=True).strip()
            return os.path.basename(repo_path)
        except:
            return "Unknown Repository"
    
    def _get_analysis_period(self, commits_data) -> str:
        """Get analysis time period"""
        if not commits_data:
            return "No data"
        
        timestamps = [c['timestamp'] for c in commits_data if c.get('timestamp')]
        if timestamps:
            start = min(timestamps).strftime('%Y-%m-%d')
            end = max(timestamps).strftime('%Y-%m-%d')
            return f"{start} to {end}"
        return "Unknown period"

# --- Main CLI Interface ---
def main():
    """Production CLI interface for DevMetrics Pro"""
    parser = argparse.ArgumentParser(
        description="DevMetrics Pro - Terminal Developer Analytics Platform",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Advanced Terminal Analytics Examples:
  python devmetrics_pro.py --since 1y                 # Full year analysis
  python devmetrics_pro.py --since 6m --width 140     # 6 months, wide display
  python devmetrics_pro.py --since 3m --height 20     # 3 months, tall charts
  python devmetrics_pro.py --export                   # Analysis + data export
        """
    )
    
    parser.add_argument("--since", default="1y", 
                       help="Analysis period: 1y, 6m, 3m, 1m, 2w, 7d")
    parser.add_argument("--width", type=int, default=120,
                       help="Terminal display width (default: 120)")
    parser.add_argument("--height", type=int, default=15,
                       help="Chart height for visualizations (default: 15)")
    parser.add_argument("--export", action="store_true",
                       help="Export detailed analytics to files")
    parser.add_argument("--export-dir", default="devmetrics_export",
                       help="Export directory path")
    parser.add_argument("--verbose", "-v", action="store_true",
                       help="Enable verbose logging output")
    
    args = parser.parse_args()
    
    # Configuration
    config = MetricsConfig(
        terminal_width=args.width,
        chart_height=args.height
    )
    
    # Setup logging
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Initialize analyzer
    analyzer = DevMetricsAnalyzer(config)
    
    try:
        # Verify git repository
        subprocess.check_output(["git", "rev-parse", "--git-dir"], 
                              stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError:
        print("ERROR: Not in a git repository")
        print("Navigate to a git repository and run the analysis again.")
        return 1
    
    try:
        # Execute analysis
        print("INITIALIZING DEVMETRICS PRO ANALYSIS...")
        print("=" * 50)
        
        commits_data = analyzer.collect_git_data(args.since)
        
        if not commits_data:
            print("ERROR: No commit data found for the specified period")
            print(f"Try: python devmetrics_pro.py --since 6m")
            return 1
        
        print("Status: Generating comprehensive analytics report...")
        print()
        
        # Generate and display report
        report = analyzer.generate_comprehensive_report(commits_data)
        print(report)
        
        # Export if requested
        if args.export:
            print("EXPORTING DETAILED ANALYTICS...")
            print("-" * 40)
            
            # Create export directory
            Path(args.export_dir).mkdir(exist_ok=True)
            
            # Export raw data
            with open(f"{args.export_dir}/raw_commits.json", 'w') as f:
                # Convert datetime objects to strings for JSON serialization
                export_commits = []
                for commit in commits_data:
                    export_commit = commit.copy()
                    if 'timestamp' in export_commit and hasattr(export_commit['timestamp'], 'isoformat'):
                        export_commit['timestamp'] = export_commit['timestamp'].isoformat()
                    export_commits.append(export_commit)
                json.dump(export_commits, f, indent=2)
            
            # Export metrics
            productivity = analyzer.calculate_productivity_metrics(commits_data)
            technical = analyzer.calculate_technical_metrics(commits_data)
            impact = analyzer.calculate_impact_metrics(commits_data)
            
            metrics_export = {
                "analysis_metadata": {
                    "generated_at": datetime.datetime.now().isoformat(),
                    "repository": analyzer._get_repo_name(),
                    "period": analyzer._get_analysis_period(commits_data),
                    "total_commits": len(commits_data)
                },
                "productivity_metrics": asdict(productivity),
                "technical_metrics": asdict(technical),
                "impact_metrics": asdict(impact)
            }
            
            with open(f"{args.export_dir}/analytics_summary.json", 'w') as f:
                json.dump(metrics_export, f, indent=2)
            
            # Export report as text file
            with open(f"{args.export_dir}/full_report.txt", 'w') as f:
                f.write(report)
            
            print(f"Analytics exported to: {args.export_dir}/")
            print("Files generated:")
            print("  - full_report.txt (Complete terminal report)")
            print("  - analytics_summary.json (Structured metrics data)")
            print("  - raw_commits.json (Detailed commit data)")
        
        print("\nANALYSIS COMPLETE")
        print(f"Total commits processed: {len(commits_data)}")
        print("Use --export flag to save detailed analytics data")
        
        return 0
        
    except KeyboardInterrupt:
        print("\nAnalysis interrupted by user")
        return 1
    except Exception as e:
        print(f"ANALYSIS ERROR: {str(e)}")
        if args.verbose:
            import traceback
            print("\nDETAILED ERROR TRACE:")
            print(traceback.format_exc())
        return 1

if __name__ == "__main__":
    exit(main())