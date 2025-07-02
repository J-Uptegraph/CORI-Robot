#!/usr/bin/env python3
"""
CORI Database Manager
Advanced database functions for object management and analytics
"""

import json
import time
import math
import os
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, asdict
from datetime import datetime
from ament_index_python.packages import get_package_share_directory

@dataclass
class ObjectData:
    """Structured object data for database operations"""
    object_name: str
    primary_location: float  # degrees
    distance: float         # meters
    confidence: float       # 0.0 to 1.0
    last_seen: float       # timestamp
    search_radius: float   # degrees
    success_count: int
    failure_count: int
    world_position: Dict[str, float]  # x, y coordinates
    notes: str = ""
    
    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON storage"""
        return asdict(self)

class CORIDatabaseManager:
    """Advanced database management for CORI spatial object tracking"""
    
    def __init__(self, database_file: str = None):
        if database_file is None:
            # Use relative path from workspace root
            workspace_root = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')
            self.database_file = os.path.join(workspace_root, 'shared', 'database', 'cori_spatial_database.json')
        else:
            self.database_file = database_file
        self.database = self.load_database()
    
    def load_database(self) -> Dict:
        """Load database with error handling and validation"""
        try:
            with open(self.database_file, 'r') as f:
                db = json.load(f)
            
            # Validate database structure
            if not self._validate_database_structure(db):
                print("⚠️  Database structure validation failed. Creating backup...")
                self._create_backup()
                
            return db
            
        except FileNotFoundError:
            print(f"❌ Database file {self.database_file} not found!")
            return self._create_empty_database()
        except json.JSONDecodeError as e:
            print(f"❌ JSON parsing error: {e}")
            print("Creating backup and starting fresh...")
            self._create_backup()
            return self._create_empty_database()
    
    def _validate_database_structure(self, db: Dict) -> bool:
        """Validate database has required structure"""
        required_keys = ["metadata", "objects"]
        if not all(key in db for key in required_keys):
            return False
        
        # Check metadata
        required_metadata = ["created", "description", "coordinate_system"]
        if not all(key in db["metadata"] for key in required_metadata):
            return False
            
        return True
    
    def _create_empty_database(self) -> Dict:
        """Create a new empty database with proper structure"""
        return {
            "metadata": {
                "created": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "description": "CORI Spatial Object Database - Maps virtual objects to angular positions",
                "coordinate_system": "Degrees from CORI center (0° = straight ahead, +° = left, -° = right)",
                "total_objects": 0,
                "version": "2.0"
            },
            "objects": {
                "red_objects": [],
                "blue_objects": [],
                "green_objects": [],
                "white_objects": [],
                "black_objects": [],
                "gray_objects": [],
                "orange_objects": []
            }
        }
    
    def _create_backup(self):
        """Create backup of current database"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_file = self.database_file.replace(".json", f"_backup_{timestamp}.json")
        try:
            with open(self.database_file, 'r') as src, open(backup_file, 'w') as dst:
                dst.write(src.read())
            print(f"💾 Backup created: {backup_file}")
        except Exception as e:
            print(f"❌ Backup failed: {e}")
    
    def save_database(self):
        """Save database with metadata updates"""
        # Update metadata
        self.database["metadata"]["total_objects"] = self.get_total_object_count()
        self.database["metadata"]["last_modified"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        try:
            with open(self.database_file, 'w') as f:
                json.dump(self.database, f, indent=2)
            print("💾 Database saved successfully")
        except Exception as e:
            print(f"❌ Save failed: {e}")
    
    def add_new_object(self, color: str, object_name: str, x: float, y: float, 
                      confidence: float = 0.8, notes: str = "") -> bool:
        """Add a new object to the database"""
        
        # Calculate angular position and distance from world coordinates
        distance = math.sqrt(x**2 + y**2)
        angle = math.degrees(math.atan2(y, x))
        
        # Create object data
        new_object = ObjectData(
            object_name=object_name,
            primary_location=angle,
            distance=distance,
            confidence=confidence,
            last_seen=time.time(),
            search_radius=10.0,
            success_count=0,
            failure_count=0,
            world_position={"x": x, "y": y},
            notes=notes or f"Added via database manager at ({x:.2f}, {y:.2f})"
        )
        
        # Add to appropriate color category
        color_key = f"{color.lower()}_objects"
        if color_key not in self.database["objects"]:
            self.database["objects"][color_key] = []
        
        # Check for duplicates
        existing_names = [obj["object_name"] for obj in self.database["objects"][color_key]]
        if object_name in existing_names:
            print(f"❌ Object '{object_name}' already exists in {color} objects")
            return False
        
        self.database["objects"][color_key].append(new_object.to_dict())
        print(f"✅ Added {object_name} to {color} objects at {angle:.1f}° (distance: {distance:.2f}m)")
        
        self.save_database()
        return True
    
    def remove_object(self, color: str, object_name: str) -> bool:
        """Remove an object from the database"""
        color_key = f"{color.lower()}_objects"
        
        if color_key not in self.database["objects"]:
            print(f"❌ No {color} objects category found")
            return False
        
        objects = self.database["objects"][color_key]
        for i, obj in enumerate(objects):
            if obj["object_name"] == object_name:
                removed_obj = objects.pop(i)
                print(f"✅ Removed {object_name} from {color} objects")
                self.save_database()
                return True
        
        print(f"❌ Object '{object_name}' not found in {color} objects")
        return False
    
    def update_object_success_rate(self, color: str, object_name: str, success: bool):
        """Update object's success/failure statistics"""
        color_key = f"{color.lower()}_objects"
        
        if color_key not in self.database["objects"]:
            return False
        
        for obj in self.database["objects"][color_key]:
            if obj["object_name"] == object_name:
                if success:
                    obj["success_count"] += 1
                else:
                    obj["failure_count"] += 1
                
                # Update confidence based on success rate
                total_attempts = obj["success_count"] + obj["failure_count"]
                success_rate = obj["success_count"] / total_attempts if total_attempts > 0 else 0
                obj["confidence"] = max(0.1, min(0.98, success_rate * 0.9 + 0.1))
                
                obj["last_seen"] = time.time()
                self.save_database()
                return True
        
        return False
    
    def get_objects_by_color(self, color: str) -> List[Dict]:
        """Retrieve all objects of a specific color"""
        color_key = f"{color.lower()}_objects"
        return self.database.get("objects", {}).get(color_key, [])
    
    def get_all_objects(self) -> Dict[str, List[Dict]]:
        """Get all objects organized by color"""
        return self.database.get("objects", {})
    
    def get_total_object_count(self) -> int:
        """Count total objects across all colors"""
        total = 0
        for color_objects in self.database.get("objects", {}).values():
            total += len(color_objects)
        return total
    
    def get_database_statistics(self) -> Dict:
        """Generate comprehensive database statistics"""
        stats = {
            "total_objects": self.get_total_object_count(),
            "objects_by_color": {},
            "average_confidence": 0,
            "most_reliable_objects": [],
            "least_reliable_objects": [],
            "database_health": "Good"
        }
        
        all_confidences = []
        all_objects_with_stats = []
        
        # Analyze by color
        for color_key, objects in self.database.get("objects", {}).items():
            color = color_key.replace("_objects", "")
            stats["objects_by_color"][color] = {
                "count": len(objects),
                "avg_confidence": 0,
                "total_successes": 0,
                "total_failures": 0
            }
            
            if objects:
                confidences = [obj["confidence"] for obj in objects]
                stats["objects_by_color"][color]["avg_confidence"] = sum(confidences) / len(confidences)
                stats["objects_by_color"][color]["total_successes"] = sum(obj["success_count"] for obj in objects)
                stats["objects_by_color"][color]["total_failures"] = sum(obj["failure_count"] for obj in objects)
                
                all_confidences.extend(confidences)
                
                # Track objects for reliability ranking
                for obj in objects:
                    total_attempts = obj["success_count"] + obj["failure_count"]
                    success_rate = obj["success_count"] / total_attempts if total_attempts > 0 else 0
                    all_objects_with_stats.append({
                        "name": obj["object_name"],
                        "color": color,
                        "success_rate": success_rate,
                        "confidence": obj["confidence"],
                        "total_attempts": total_attempts
                    })
        
        # Overall statistics
        if all_confidences:
            stats["average_confidence"] = sum(all_confidences) / len(all_confidences)
        
        # Reliability rankings (only objects with at least 3 attempts)
        reliable_objects = [obj for obj in all_objects_with_stats if obj["total_attempts"] >= 3]
        reliable_objects.sort(key=lambda x: x["success_rate"], reverse=True)
        
        stats["most_reliable_objects"] = reliable_objects[:5]
        stats["least_reliable_objects"] = reliable_objects[-5:] if len(reliable_objects) > 5 else []
        
        # Database health assessment
        if stats["average_confidence"] < 0.6:
            stats["database_health"] = "Poor - Low confidence scores"
        elif stats["total_objects"] < 3:
            stats["database_health"] = "Warning - Few objects tracked"
        
        return stats
    
    def print_database_report(self):
        """Print a comprehensive database report"""
        stats = self.get_database_statistics()
        
        print("\n" + "="*60)
        print("📊 CORI DATABASE ANALYTICS REPORT")
        print("="*60)
        
        print(f"\n📈 OVERVIEW:")
        print(f"   Total Objects: {stats['total_objects']}")
        print(f"   Average Confidence: {stats['average_confidence']:.2f}")
        print(f"   Database Health: {stats['database_health']}")
        
        print(f"\n🎨 OBJECTS BY COLOR:")
        for color, data in stats['objects_by_color'].items():
            if data['count'] > 0:
                success_rate = data['total_successes'] / (data['total_successes'] + data['total_failures']) if (data['total_successes'] + data['total_failures']) > 0 else 0
                print(f"   {color.title()}: {data['count']} objects, "
                      f"avg confidence: {data['avg_confidence']:.2f}, "
                      f"success rate: {success_rate:.1%}")
        
        if stats['most_reliable_objects']:
            print(f"\n🏆 MOST RELIABLE OBJECTS:")
            for obj in stats['most_reliable_objects']:
                print(f"   {obj['name']} ({obj['color']}): {obj['success_rate']:.1%} success rate")
        
        if stats['least_reliable_objects']:
            print(f"\n⚠️  OBJECTS NEEDING ATTENTION:")
            for obj in stats['least_reliable_objects']:
                print(f"   {obj['name']} ({obj['color']}): {obj['success_rate']:.1%} success rate")
        
        print("\n" + "="*60)

def main():
    """Interactive database management interface"""
    db_manager = CORIDatabaseManager()
    
    while True:
        print("\n" + "="*50)
        print("🗄️  CORI DATABASE MANAGER")
        print("="*50)
        print("1. View database report")
        print("2. Add new object")
        print("3. Remove object") 
        print("4. List objects by color")
        print("5. Update object statistics")
        print("6. Create database backup")
        print("7. Exit")
        
        choice = input("\nSelect option (1-7): ").strip()
        
        if choice == "1":
            db_manager.print_database_report()
            
        elif choice == "2":
            color = input("Object color: ").strip().lower()
            name = input("Object name: ").strip()
            try:
                x = float(input("X coordinate (meters): "))
                y = float(input("Y coordinate (meters): "))
                confidence = float(input("Initial confidence (0.0-1.0, default 0.8): ") or "0.8")
                notes = input("Notes (optional): ").strip()
                
                db_manager.add_new_object(color, name, x, y, confidence, notes)
            except ValueError:
                print("❌ Invalid input. Please enter numeric values for coordinates.")
                
        elif choice == "3":
            color = input("Object color: ").strip().lower()
            name = input("Object name: ").strip()
            db_manager.remove_object(color, name)
            
        elif choice == "4":
            color = input("Color to list: ").strip().lower()
            objects = db_manager.get_objects_by_color(color)
            if objects:
                print(f"\n{color.title()} objects:")
                for obj in objects:
                    print(f"  • {obj['object_name']} at {obj['primary_location']:.1f}° "
                          f"(confidence: {obj['confidence']:.2f})")
            else:
                print(f"No {color} objects found")
                
        elif choice == "5":
            color = input("Object color: ").strip().lower()
            name = input("Object name: ").strip()
            success = input("Was last detection successful? (y/n): ").strip().lower() == 'y'
            db_manager.update_object_success_rate(color, name, success)
            
        elif choice == "6":
            db_manager._create_backup()
            
        elif choice == "7":
            print("👋 Goodbye!")
            break
            
        else:
            print("❌ Invalid option. Please try again.")

if __name__ == "__main__":
    main()
