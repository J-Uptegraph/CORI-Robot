import json
import time
import random
import datetime
import os
from typing import Dict, List, Optional, Tuple, Union
from dataclasses import dataclass

@dataclass
class DetectedItem:
    color: str
    confidence: float
    timestamp: float

class CORILaundryAssistant:
    def __init__(self, database_file: str = "../../database/cori_spatial_database.json"):
        self.database_file = database_file
        self.database = self.load_database()
        self.initialize_laundry_system()
        self.current_user = ""
        
    def initialize_laundry_system(self):
        """Initialize laundry-specific database structure"""
        if "laundry_data" not in self.database:
            self.database["laundry_data"] = {
                "total_items_sorted": 0,
                "hamper_counts": {"Lights": 0, "Darks": 0, "Colors": 0},
                "learning_database": {},
                "user_corrections": [],
                "confidence_level": "learning",
                "custom_colors": {},
                "sorted_items_history": [],
                "user_profile": {}
            }
            self.save_database()
    
    def load_database(self) -> Dict:
        """Load the CORI spatial database"""
        database_dir = os.path.dirname(self.database_file)
        if database_dir and not os.path.exists(database_dir):
            os.makedirs(database_dir)
            
        default_structure = {
            "metadata": {
                "created": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "description": "CORI Spatial Object Database",
                "version": "3.0"
            },
            "objects": {},
            "laundry_data": {
                "total_items_sorted": 0,
                "hamper_counts": {"Lights": 0, "Darks": 0, "Colors": 0},
                "learning_database": {},
                "user_corrections": [],
                "confidence_level": "learning",
                "custom_colors": {},
                "sorted_items_history": [],
                "user_profile": {}
            }
        }
        
        try:
            with open(self.database_file, 'r') as f:
                database = json.load(f)
                # Ensure structure is complete
                if "laundry_data" not in database:
                    database["laundry_data"] = default_structure["laundry_data"]
                return database
        except (FileNotFoundError, json.JSONDecodeError) as e:
            if isinstance(e, json.JSONDecodeError):
                print("⚠️  Database file corrupted. Creating new one.")
            return default_structure
    
    def save_database(self) -> bool:
        """Save the database back to file"""
        database_dir = os.path.dirname(self.database_file)
        if database_dir and not os.path.exists(database_dir):
            os.makedirs(database_dir)
        try:
            with open(self.database_file, 'w') as f:
                json.dump(self.database, f, indent=2)
            return True
        except Exception as e:
            print(f"⚠️  Could not save database: {e}")
            return False
    
    def get_learning_phase(self) -> str:
        """Determine CORI's current learning phase"""
        total_sorted = self.database["laundry_data"]["total_items_sorted"]
        correction_count = len(self.database["laundry_data"]["user_corrections"])
        
        # Factor in corrections - too many corrections means we need more learning
        if total_sorted < 15 or correction_count > total_sorted * 0.3:
            return "learning"
        elif total_sorted < 30 or correction_count > total_sorted * 0.2:
            return "tentative"
        else:
            return "confident"
    
    def display_banner(self, message: str = ""):
        """Display the CORI ASCII art banner with proper formatting"""
        BANNER_WIDTH = 62
        CONTENT_WIDTH = BANNER_WIDTH - 2  # Account for border characters
        
        banner_lines = [
            "╔════════════════════════════════════════════════════════════╗",
            "║                                                            ║", 
            "║         ██████╗    ██████╗     ██████╗    ██╗             ║",
            "║         ██╔════╝   ██╔═══██╗   ██╔══██╗   ██║             ║",
            "║         ██║        ██║   ██║   ██████╔╝   ██║             ║",
            "║         ██║        ██║   ██║   ██╔══██╗   ██║             ║",
            "║         ╚██████╗   ╚██████╔╝   ██║  ██║   ██║             ║",
            "║          ╚═════╝    ╚═════╝    ╚═╝  ╚═╝   ╚═╝             ║",
            "║                                                            ║", 
            "║     Cooperative Organizational Robotic Intelligence       ║",
            "║               Laundry Sorting Assistant                    ║",
            "║                                                            ║", 
            "║         \"Built to function, Designed to Matter\"          ║",
            "║           - Developed by Johnathan Uptegraph              ║", 
            "║                                                            ║"
        ]

        # Add custom message if provided
        if message:
            padded_message = message.center(CONTENT_WIDTH)
            banner_lines.append(f"║{padded_message}║")
            banner_lines.append("║                                                            ║")
        
        # Add stats line
        counts = self.database["laundry_data"]["hamper_counts"]
        total = self.database["laundry_data"]["total_items_sorted"]
        date_time = datetime.datetime.now().strftime('%I:%M %p EDT, %B %d, %Y')
        
        stats_line = f"Sorted: {total} | L:{counts['Lights']} D:{counts['Darks']} C:{counts['Colors']}"
        
        # Ensure stats line fits
        if len(stats_line) + len(date_time) + 3 <= CONTENT_WIDTH:
            stats_line = f"{stats_line} | {date_time}"
        elif len(stats_line) <= CONTENT_WIDTH:
            pass  # Just use stats without date
        else:
            stats_line = f"Total: {total} | L:{counts['Lights']} D:{counts['Darks']} C:{counts['Colors']}"
        
        padded_stats = stats_line.center(CONTENT_WIDTH)
        banner_lines.append(f"║{padded_stats}║")
        banner_lines.append("║                                                            ║")
        
        # Add status line
        status = f"Status: {self.get_learning_phase().title()} Mode"
        padded_status = status.center(CONTENT_WIDTH)
        banner_lines.append(f"║{padded_status}║")
        banner_lines.append("╚════════════════════════════════════════════════════════════╝")
        
        for line in banner_lines:
            print(line)
        time.sleep(0.1)

    def show_startup_sequence(self) -> bool:
        """Show startup sequence and handle user authentication"""
        print("\n" + "═" * 79)
        print("Initializing C.O.R.I. System", end="")
        for _ in range(3):
            time.sleep(0.5)
            print(".", end="", flush=True)
        print("\n" + "═" * 79)

        print("\n🤖 Hi there! I'm C.O.R.I., your Cooperative Organizational Robotic Intelligence!")
        print("📋 Now that you know who I am, can you tell me who I'm working with today?")
        
        user_name = input("\n> ").strip()
        if not user_name:
            user_name = "User"
        
        self.current_user = user_name.lower()
        
        print(f"\n👋 Hi {user_name.title()}! Are we sorting your laundry today?")
        
        is_existing_user = False
        
        if self._get_yes_no_input():
            is_existing_user = self._load_user_profile(self.current_user)
            if not is_existing_user:
                print("\n📝 I don't have a profile for you yet. Let me create one...")
                self._create_user_profile(self.current_user)
        else:
            print("\n❓ Who are we sorting for today then?")
            other_name = input("Enter name: ").strip()
            if other_name:
                self.current_user = other_name.lower()
                is_existing_user = self._load_user_profile(self.current_user)
                if not is_existing_user:
                    print("\n📝 Creating a new profile...")
                    self._create_user_profile(self.current_user)
        
        # Show appropriate welcome message and launch menu
        self._display_welcome_and_launch_menu(is_existing_user)
        return True

    def _display_welcome_and_launch_menu(self, is_existing_user: bool):
        """Display welcome message based on user status and launch menu"""
        if is_existing_user:
            print(f"\n🎉 Welcome back! Great to see you again.")
        else:
            print(f"\n🌟 Welcome! I'm excited to help you get organized.")
        
        print("💡 What can I help you with today?")
        print()
        
        # Launch main menu immediately
        self.show_main_menu()

    def _get_yes_no_input(self, prompt: str = "(y/n): ") -> bool:
        """Get standardized yes/no input from user"""
        while True:
            response = input(prompt).strip().lower()
            if response in ['y', 'yes']:
                return True
            elif response in ['n', 'no']:
                return False
            else:
                print("Please enter 'y' for yes or 'n' for no.")

    def _load_user_profile(self, name: str) -> bool:
        """Load user profile from database or archived files"""
        # Check current database first
        if name in self.database["laundry_data"]["user_profile"]:
            print(f"\n✅ Found your profile! Welcome back, {name.title()}!")
            self.database["laundry_data"]["user_profile"][name]["last_used"] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.save_database()
            self.display_banner(f"LOADING {name.title()}'S PROFILE...")
            return True
        
        # Search archived database files
        database_dir = os.path.dirname(self.database_file)
        if os.path.exists(database_dir):
            for filename in os.listdir(database_dir):
                if filename.endswith(".json") and "cori_spatial_database" in filename:
                    try:
                        with open(os.path.join(database_dir, filename), 'r') as f:
                            saved_data = json.load(f)
                            if "laundry_data" in saved_data and name in saved_data["laundry_data"].get("user_profile", {}):
                                print(f"\n🔍 Found a profile for {name.title()} in archived data!")
                                print("Would you like to load this profile?")
                                if self._get_yes_no_input():
                                    self.database = saved_data
                                    self.database["laundry_data"]["user_profile"][name]["last_used"] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                                    self.save_database()
                                    self.display_banner(f"LOADING {name.title()}'S PROFILE...")
                                    return True
                    except (json.JSONDecodeError, Exception):
                        continue
        return False

    def _create_user_profile(self, name: str):
        """Create a new user profile"""
        if "user_profile" not in self.database["laundry_data"]:
            self.database["laundry_data"]["user_profile"] = {}
            
        self.database["laundry_data"]["user_profile"][name] = {
            "created": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "last_used": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        self.save_database()
        self.display_banner(f"CREATING {name.title()}'S PROFILE...")

    def _display_post_startup_messages(self):
        """Display messages after startup sequence"""
        total_sorted = self.database["laundry_data"]["total_items_sorted"]
        
        print("\n🎓 I'm always learning! Please help me by correcting any mistakes.")
        print("─" * 60)
        print("🧺 Ready to sort your laundry! ✨")
        print("─" * 60)
        
        if total_sorted == 0:
            print("🎉 Looks like this is your first time! Welcome aboard!")
        elif total_sorted < 15:
            remaining = 15 - total_sorted
            print(f"📚 I'm still learning your style. {remaining} more items until I get confident!")
        elif total_sorted < 30:
            remaining = 30 - total_sorted
            print(f"🤖 Getting smarter! {remaining} more items until I'm an expert.")
        else:
            print("🎓 I'm a laundry sorting expert with your preferences!")
        print()

    def normalize_color_input(self, color_input: str) -> str:
        """Normalize color input handling variations and common typos"""
        color_input = color_input.lower().strip()
        
        # Handle underscores and spaces
        color_input = color_input.replace('_', ' ').replace('-', ' ')
        
        # Remove extra spaces
        color_input = ' '.join(color_input.split())
        
        # Common corrections
        corrections = {
            'light grey': ['light gray', 'lightgrey', 'lightgray'],
            'dark grey': ['dark gray', 'darkgrey', 'darkgray'],
            'light pink': ['lightpink'],
            'dark blue': ['darkblue'],
            'dark green': ['darkgreen'],
            'bright pink': ['brightpink'],
            'bright blue': ['brightblue'],
            'bright green': ['brightgreen'],
            'yellow': ['yelow', 'yelllow'],
            'purple': ['purpel', 'purpal'],
            'orange': ['ornage', 'oragne'],
            'cream': ['creme'],
            'navy': ['navey', 'nafy']
        }
        
        # Apply corrections
        for correct, variants in corrections.items():
            if color_input in variants:
                return correct
                
        return color_input

    def get_known_colors(self) -> List[str]:
        """Get all known colors from database and defaults"""
        default_colors = [
            'white', 'light grey', 'tan', 'yellow', 'cream', 'light pink',
            'red', 'blue', 'green', 'orange', 'purple', 'bright pink',
            'black', 'dark grey', 'navy', 'dark blue', 'brown', 'dark green'
        ]
        
        custom_colors = list(self.database["laundry_data"].get("custom_colors", {}).keys())
        return default_colors + custom_colors

    def get_default_category(self, color: str) -> str:
        """Get default laundry category based on color"""
        base_color = color.split('#')[0] if '#' in color else color
        
        # Check custom colors first
        custom_colors = self.database["laundry_data"].get("custom_colors", {})
        if base_color in custom_colors:
            return custom_colors[base_color]
        
        # Default categorization
        lights = ['white', 'light grey', 'tan', 'yellow', 'cream', 'light pink']
        darks = ['black', 'dark grey', 'navy', 'dark blue', 'brown', 'dark green']
        
        if base_color in lights:
            return "Lights"
        elif base_color in darks:
            return "Darks"
        else:
            return "Colors"

    def get_learned_category(self, color: str) -> Optional[str]:
        """Get category that user has specifically taught for this color"""
        base_color = color.split('#')[0] if '#' in color else color
        return self.database["laundry_data"]["learning_database"].get(base_color)

    def update_learned_category(self, color: str, category: str):
        """Update the learned category for a color"""
        base_color = color.split('#')[0] if '#' in color else color
        self.database["laundry_data"]["learning_database"][base_color] = category

    def parse_item_input(self, item_input: str) -> Tuple[Optional[str], str]:
        """Parse combined color + clothing input"""
        if not item_input or item_input.strip() == '':
            return None, ""
        
        item_input = item_input.strip().lower()
        
        clothing_types = [
            'shirt', 'shirts', 'pants', 'jeans', 'shorts', 'sock', 'socks', 'dress', 'skirt', 
            'jacket', 'coat', 'sweater', 'hoodie', 'top', 'blouse', 'tank', 'tee', 't-shirt', 
            'polo', 'cardigan', 'vest', 'underwear', 'bra', 'boxers', 'briefs', 'pajamas', 
            'pjs', 'sleepwear', 'towel', 'sheets', 'pillowcase', 'blanket'
        ]
        
        words = item_input.split()
        clothing_type = "item"
        color_words = []
        
        # Find clothing type and extract color words
        for word in words:
            if word in clothing_types:
                clothing_type = word
            else:
                color_words.append(word)
        
        color_input = " ".join(color_words).strip()
        
        # If no clear separation, assume first word is color, rest is clothing
        if not color_input and len(words) >= 2:
            color_input = words[0]
            clothing_type = " ".join(words[1:])
        elif not color_input:
            color_input = item_input
            clothing_type = "item"
        
        return color_input if color_input else None, clothing_type

    def define_new_color(self, color_input: str) -> Optional[str]:
        """Handle definition of new colors by user"""
        print(f"\n🎨 I don't recognize the color '{color_input}' yet!")
        print("Would you like to teach me about this color?")
        
        if not self._get_yes_no_input("Teach me this color? (y/n): "):
            print("💡 No problem! Try using a color I already know.")
            return None
        
        print(f"\n🤔 Where should '{color_input}' items usually go?")
        print("   1) Lights (whites, creams, light colors)")
        print("   2) Darks (blacks, dark colors that might bleed)")  
        print("   3) Colors (bright colors, medium tones)")
        
        while True:
            choice = input("\nChoose 1, 2, or 3: ").strip()
            if choice == "1":
                category = "Lights"
                break
            elif choice == "2":
                category = "Darks"
                break
            elif choice == "3":
                category = "Colors"
                break
            else:
                print("⚠️  Please choose 1, 2, or 3")
        
        # Store custom color
        if "custom_colors" not in self.database["laundry_data"]:
            self.database["laundry_data"]["custom_colors"] = {}
        
        self.database["laundry_data"]["custom_colors"][color_input] = category
        self.save_database()
        
        print(f"✅ Perfect! I'll remember that '{color_input}' goes in {category}")
        return category

    def get_cori_reasoning(self, color: str, category: str) -> str:
        """Generate CORI's reasoning for sorting decision"""
        base_color = color.split('#')[0] if '#' in color else color
        
        # Check if it's a custom color
        custom_colors = self.database["laundry_data"].get("custom_colors", {})
        if base_color in custom_colors:
            return f"You taught me that {base_color} items go in {category}"
        
        # Default reasoning
        reasoning_map = {
            "Lights": {
                "white": "White items need to stay bright and clean!",
                "light grey": "Light colors should stay with other lights to avoid getting dingy",
                "tan": "Tan is delicate and light - safer with whites",
                "yellow": "Yellow can get muddy with dark colors",
                "cream": "Cream is basically white - definitely goes with lights",
                "light pink": "Light pink is too delicate for other loads"
            },
            "Darks": {
                "black": "Black is the darkest - definitely goes with darks",
                "dark grey": "Dark grey might leave lint on lighter items",
                "navy": "Navy is dark enough to potentially bleed color",
                "dark blue": "Dark blue should stay with other dark items",
                "brown": "Brown is a heavy, dark color",
                "dark green": "Dark green is too intense for the color load"
            },
            "Colors": {
                "red": "Red has good color but isn't dark enough for darks",
                "blue": "Blue is bright but not too light or too dark",
                "green": "Green is a nice middle-ground color",
                "orange": "Orange is vibrant but not bleeding-level dark",
                "purple": "Purple is colorful but not delicate like lights",
                "bright pink": "Bright pink has personality - goes with other colors"
            }
        }
        
        return reasoning_map.get(category, {}).get(base_color, f"{base_color} seems like a good fit for {category}")

    def detect_clothing_item(self) -> Optional[DetectedItem]:
        """Get clothing item input from user with natural prompts"""
        total_sorted = self.database["laundry_data"]["total_items_sorted"]
        
        # Show encouragement periodically
        if total_sorted > 0 and total_sorted % 10 == 0:
            encouragements = [
                f"🎉 Great work! We've sorted {total_sorted} items together.",
                f"👏 You're getting the hang of this! {total_sorted} items done.",
                f"🤝 Excellent teamwork! {total_sorted} items sorted so far.",
                f"🚀 Look at us go! {total_sorted} items completed."
            ]
            print(f"\n{random.choice(encouragements)}")
        
        # Choose appropriate prompt
        prompts = [
            "What's next?", "Show me another item!", "What do you have for me?",
            "Ready for the next one!", "Let's keep going - what's this?", 
            "What are we sorting now?", "Show me what you've got!"
        ]
        
        prompt = "🧺 What's the first item?" if total_sorted == 0 else f"🔍 {random.choice(prompts)}"
        
        print(f"\n{prompt}")
        print("💡 Examples: 'red shirt', 'blue jeans' | Commands: 'help', 'stats', 'menu', 'quit'")
        
        while True:
            item_input = input("\n> ").strip().lower()
            
            if item_input in ['quit', 'exit']:
                return None
            elif item_input == 'menu':
                self.show_main_menu()
                continue
            elif item_input in ['help', '?']:
                self.show_help()
                continue
            elif item_input == 'stats':
                self.show_quick_stats()
                continue
            elif item_input == '':
                print("💭 Please tell me what item you'd like to sort!")
                continue
            else:
                return self.process_item_input(item_input)

    def process_item_input(self, item_input: str) -> Optional[DetectedItem]:
        """Process user's item input and create DetectedItem"""
        color_input, clothing_type = self.parse_item_input(item_input)
        
        if not color_input:
            responses = [
                "🤔 I didn't catch that. Try 'red shirt' or 'blue pants'",
                "❓ Could you be more specific? Like 'green sweater'?",
                "📝 I need a color and item type - like 'white socks'",
                "🎯 What color is it? And what kind of clothing?"
            ]
            print(f"{random.choice(responses)}")
            return None
        
        # Normalize the color
        color_input = self.normalize_color_input(color_input)
        
        # Check if color is known
        known_colors = self.get_known_colors()
        if color_input not in known_colors:
            category = self.define_new_color(color_input)
            if not category:
                return None
        
        # Confirmation message
        detection_responses = [
            f"👀 Ah, a {color_input} {clothing_type}!",
            f"✨ I see a {color_input} {clothing_type}.",
            f"🎯 Got it - {color_input} {clothing_type}!",
            f"📋 Nice, a {color_input} {clothing_type}."
        ]
        print(f"{random.choice(detection_responses)}")
        
        time.sleep(0.3)  # Brief pause for natural feel
        
        return DetectedItem(
            color=f"{color_input}#{clothing_type}",
            confidence=random.uniform(0.85, 0.98),
            timestamp=time.time()
        )

    def make_sorting_decision(self, item: DetectedItem) -> Tuple[str, str, bool]:
        """Make sorting decision based on current learning phase"""
        phase = self.get_learning_phase()
        color = item.color
        base_color = color.split('#')[0] if '#' in color else color
        clothing_type = color.split('#')[1] if '#' in color else 'item'
        
        learned_category = self.get_learned_category(base_color)
        was_corrected = False
        
        print(f"\n🤖 Processing {base_color} {clothing_type}...")
        
        if phase == "learning":
            # Learning phase: Always ask or use known preferences
            if learned_category:
                print(f"💭 I remember putting {base_color} items in {learned_category} before.")
                print(f"Should this {clothing_type} go there too?")
                if self._get_yes_no_input():
                    return learned_category, "following your previous choice", False
            
            # Ask user where it should go
            print(f"🎓 Where should this {base_color} {clothing_type} go?")
            category = self._get_category_choice()
            reason = "you taught me"
            was_corrected = True
            
        elif phase == "tentative":
            # Tentative phase: Make suggestions and ask for confirmation
            if learned_category:
                suggested = learned_category
                print(f"💡 Based on your style, this probably goes in {suggested}.")
                reason = "following your pattern"
            else:
                suggested = self.get_default_category(color)
                reasoning = self.get_cori_reasoning(color, suggested)
                print(f"💡 I'm thinking {suggested}.")
                print(f"🧠 {reasoning}")
                reason = reasoning
            
            print("Does that sound right?")
            if self._get_yes_no_input():
                return suggested, reason, False
            else:
                print("🤔 Okay, where would you put it instead?")
                category = self._get_category_choice()
                correction_reason = input("Why there? ").strip()
                self.record_correction(base_color, suggested, category, correction_reason)
                reason = f"you corrected me: {correction_reason}"
                was_corrected = True
                
        else:  # confident phase
            # Confident phase: Make decisions but allow overrides
            if learned_category:
                category = learned_category
                confident_statements = [
                    f"✅ Easy - {category}!",
                    f"🎯 This goes in {category} for sure.",
                    f"💪 Definitely {category}.",
                    f"🎪 {category}, no question."
                ]
                print(f"{random.choice(confident_statements)}")
                print(f"💭 That's where all your {base_color} items go.")
                reason = "following your established pattern"
            else:
                category = self.get_default_category(color)
                reasoning = self.get_cori_reasoning(color, category)
                confident_statements = [
                    f"✅ I'm putting this in {category}.",
                    f"🎯 This belongs in {category}.",
                    f"💪 {category} is the right choice."
                ]
                print(f"{random.choice(confident_statements)}")
                print(f"🧠 {reasoning}")
                reason = reasoning
            
            print("Need to override my decision?")
            if self._get_yes_no_input():
                print("🔄 Where should it go instead?")
                new_category = self._get_category_choice()
                correction_reason = input("Why there? ").strip()
                self.record_correction(base_color, category, new_category, correction_reason)
                category = new_category
                reason = f"you corrected me: {correction_reason}"
                was_corrected = True
        
        return category, reason, was_corrected

    def _get_category_choice(self) -> str:
        """Get category choice from user with standardized input"""
        print("   1) Lights")
        print("   2) Darks")
        print("   3) Colors")
        
        while True:
            choice = input("\nChoose 1, 2, or 3: ").strip().lower()
            if choice in ["1", "lights"]:
                return "Lights"
            elif choice in ["2", "darks"]:
                return "Darks"
            elif choice in ["3", "colors"]:
                return "Colors"
            else:
                print("⚠️  Please choose 1 (Lights), 2 (Darks), or 3 (Colors)")

    def record_correction(self, color: str, wrong_category: str, right_category: str, reason: str):
        """Record user correction and update learning database"""
        correction = {
            "color": color,
            "wrong_category": wrong_category,
            "right_category": right_category,
            "reason": reason,
            "timestamp": time.time()
        }
        
        self.database["laundry_data"]["user_corrections"].append(correction)
        self.update_learned_category(color, right_category)
        
        responses = [
            f"📝 Got it! {color.title()} items go in {right_category}. I'll remember that.",
            f"✅ Thanks for the correction! {color.title()} → {right_category}. Makes sense!",
            f"🎓 Good to know! I'll put {color} items in {right_category} from now on.",
            f"💡 Perfect! {color.title()} goes in {right_category}. Learning something new!"
        ]
        print(f"{random.choice(responses)}")

    def update_laundry_counts(self, category: str, item: DetectedItem):
        """Update hamper counts and learning database"""
        base_color = item.color.split('#')[0] if '#' in item.color else item.color
        clothing_type = item.color.split('#')[1] if '#' in item.color else 'item'
        item_name = f"{base_color.title()} {clothing_type.title()}"
        
        # Update counts
        self.database["laundry_data"]["hamper_counts"][category] += 1
        self.database["laundry_data"]["total_items_sorted"] += 1
        
        # Update learning database only if this was a user-taught decision
        # (This prevents overriding corrections with default categories)
        
        # Add to history
        if "sorted_items_history" not in self.database["laundry_data"]:
            self.database["laundry_data"]["sorted_items_history"] = []
        
        item_record = {
            "item_name": f"{item_name} ({self.current_user.title()})",
            "category": category,
            "timestamp": item.timestamp,
            "color": base_color
        }
        self.database["laundry_data"]["sorted_items_history"].append(item_record)
        self.save_database()

    def generate_sorting_report(self, item: DetectedItem, category: str, reason: str):
        """Generate natural sorting report with progress updates"""
        clothing_name = item.color.replace('#', ' ').title()
        
        # Completion message
        completion_messages = [
            f"✅ Done! The {clothing_name} is now in the {category} hamper.",
            f"🎯 Perfect! {clothing_name} → {category} hamper.",
            f"✨ All set! Your {clothing_name} is sorted into {category}.",
            f"🧺 There we go! {clothing_name} goes in {category}."
        ]
        print(f"\n{random.choice(completion_messages)}")
        
        # Show reasoning if informative
        if reason and not reason.startswith("following your"):
            print(f"💡 {reason}")
        
        # Current stats
        counts = self.database["laundry_data"]["hamper_counts"]
        total = sum(counts.values())
        print(f"\n📊 Current hampers: Lights({counts['Lights']}) | Darks({counts['Darks']}) | Colors({counts['Colors']})")
        
        # Learning progress
        phase = self.get_learning_phase()
        correction_rate = len(self.database["laundry_data"]["user_corrections"]) / max(total, 1)
        
        if phase == "learning":
            if total < 15:
                remaining = 15 - total
                remaining_text = "item" if remaining == 1 else "items"
                print(f"🧠 Still learning your style! ({remaining} more {remaining_text} until I get confident)")
            else:
                print(f"⚠️  I need fewer corrections to advance. Keep teaching me your preferences!")
        elif phase == "tentative":
            if total < 30 and correction_rate <= 0.2:
                remaining = 30 - total
                remaining_text = "item" if remaining == 1 else "items"
                print(f"🤖 Getting the hang of this! ({remaining} more {remaining_text} until I'm fully confident)")
            else:
                print(f"🎓 Making progress! I'll get more confident as I learn your style better.")
        else:
            print(f"🎓 I've got your laundry style down! Making confident decisions now.")
        
        # Milestone celebrations
        if total == 15 and phase != "learning":
            print(f"🎉 Milestone! I'm starting to understand your preferences!")
        elif total == 30 and phase == "confident":
            print(f"🎊 Expert level achieved! I've mastered your laundry style!")

    def show_quick_stats(self):
        """Show quick stats without full menu"""
        counts = self.database["laundry_data"]["hamper_counts"]
        total = self.database["laundry_data"]["total_items_sorted"]
        phase = self.get_learning_phase()
        correction_count = len(self.database["laundry_data"]["user_corrections"])
        
        items_text = "item" if total == 1 else "items"
        print(f"\n📊 QUICK STATS")
        print(f"─" * 30)
        print(f"📦 Total sorted: {total} {items_text}")
        print(f"🤍 Lights: {counts['Lights']} | ⚫ Darks: {counts['Darks']} | 🌈 Colors: {counts['Colors']}")
        print(f"🎓 Learning phase: {phase.title()}")
        print(f"📝 Corrections: {correction_count}")
        
        if phase == "learning":
            remaining = max(0, 15 - total)
            if remaining > 0:
                print(f"🎯 {remaining} more items until tentative mode")
        elif phase == "tentative":
            remaining = max(0, 30 - total)
            if remaining > 0:
                print(f"🎯 {remaining} more items until confident mode")

    def show_help(self):
        """Show help information"""
        print(f"\n💡 HELP - HOW TO USE C.O.R.I.")
        print(f"═" * 40)
        print(f"\n🎯 TELLING ME ABOUT ITEMS:")
        print(f"   • Just describe what you see: 'red shirt', 'blue jeans'")
        print(f"   • I understand most clothing types and colors")
        print(f"   • If I don't know a color, I'll ask you to teach me!")
        
        print(f"\n🗂️  THE THREE HAMPERS:")
        print(f"   • 🤍 LIGHTS: Whites, creams, light colors")
        print(f"   • ⚫ DARKS: Blacks, navy, dark colors that might bleed")
        print(f"   • 🌈 COLORS: Bright colors, medium tones")
        
        print(f"\n🧠 HOW I LEARN:")
        print(f"   • Phase 1 (Items 1-15): I ask about everything")
        print(f"   • Phase 2 (Items 16-30): I suggest, you approve")
        print(f"   • Phase 3 (Items 31+): I decide confidently, you can override")
        
        print(f"\n⌨️  COMMANDS:")
        print(f"   • 'menu' - Show main menu")
        print(f"   • 'stats' - Quick statistics")
        print(f"   • 'help' - This help screen")
        print(f"   • 'quit' - Exit program")
        
        input(f"\nPress enter to continue...")

    def show_main_menu(self):
        """Show main menu and handle navigation"""
        total_sorted = self.database["laundry_data"]["total_items_sorted"]
        counts = self.database["laundry_data"]["hamper_counts"]
        phase = self.get_learning_phase()
        
        print(f"\n🤖 C.O.R.I. MAIN MENU")
        print(f"═" * 40)
        
        items_text = "item" if total_sorted == 1 else "items"
        print(f"📊 Status: {total_sorted} {items_text} sorted | {phase.title()} mode")
        print(f"📦 Hampers: Lights({counts['Lights']}) | Darks({counts['Darks']}) | Colors({counts['Colors']})")
        
        # Show custom colors if any
        custom_colors = list(self.database["laundry_data"].get("custom_colors", {}).keys())
        if custom_colors:
            print(f"🎨 Custom colors: {', '.join(custom_colors)}")
        
        print(f"\nWhat would you like to do?")
        print(f"   1) Continue sorting items")
        print(f"   2) View hamper contents")
        print(f"   3) Show detailed statistics")
        print(f"   4) Start fresh session")
        print(f"   5) Help & instructions")
        print(f"   6) Quit")
        
        while True:
            choice = input(f"\nChoose 1-6: ").strip().lower()
            
            if choice in ['1', 'continue', 'sort', '']:
                return
            elif choice in ['2', 'hampers', 'contents']:
                self.show_hamper_menu()
                return
            elif choice in ['3', 'stats', 'statistics']:
                self.show_detailed_stats()
                input("\nPress enter to continue...")
                return
            elif choice in ['4', 'fresh', 'reset', 'new']:
                if self.create_blank_database():
                    print("\n🆕 Fresh start initiated!")
                return
            elif choice in ['5', 'help']:
                self.show_help()
                return
            elif choice in ['6', 'quit', 'exit']:
                self.quit_program()
                return
            else:
                print("⚠️  Please choose a number from 1-6")

    def show_hamper_menu(self):
        """Show hamper contents menu"""
        print(f"\n📦 HAMPER CONTENTS")
        print(f"═" * 30)
        print(f"Which hamper would you like to see?")
        print(f"   1) Lights")
        print(f"   2) Darks")
        print(f"   3) Colors")
        print(f"   4) All hampers")
        print(f"   5) Back to main menu")
        
        while True:
            choice = input(f"\nChoose 1-5: ").strip().lower()
            
            if choice in ['1', 'lights']:
                self.show_hamper_contents("Lights")
                break
            elif choice in ['2', 'darks']:
                self.show_hamper_contents("Darks")
                break
            elif choice in ['3', 'colors']:
                self.show_hamper_contents("Colors")
                break
            elif choice in ['4', 'all']:
                self.show_all_hamper_contents()
                break
            elif choice in ['5', 'back', 'menu']:
                return
            else:
                print("⚠️  Please choose a number from 1-5")
        
        input("\nPress enter to continue...")

    def show_hamper_contents(self, hamper_name: str):
        """Show contents of a specific hamper"""
        items_in_hamper = [
            item for item in self.database["laundry_data"].get("sorted_items_history", [])
            if item.get("category") == hamper_name
        ]
        
        hamper_count = self.database["laundry_data"]["hamper_counts"].get(hamper_name, 0)
        
        print(f"\n📦 {hamper_name.upper()} HAMPER")
        print(f"─" * (len(hamper_name) + 10))
        
        if hamper_count == 0:
            print(f"   📭 Empty - no {hamper_name.lower()} items sorted yet")
        elif not items_in_hamper and hamper_count > 0:
            items_text = "item" if hamper_count == 1 else "items"
            print(f"   📊 {hamper_count} {items_text} (from before detailed tracking)")
        else:
            items_text = "item" if len(items_in_hamper) == 1 else "items"
            print(f"   📊 {len(items_in_hamper)} {items_text} with details:")
            
            # Show recent items (last 10)
            recent_items = items_in_hamper[-10:]
            for i, item in enumerate(recent_items, 1):
                item_name = item.get("item_name", "Unknown item")
                timestamp = item.get("timestamp", 0)
                time_str = datetime.datetime.fromtimestamp(timestamp).strftime("%m/%d %H:%M")
                print(f"   {i:2d}. {item_name} (sorted {time_str})")
            
            # Show count of older items if any
            if len(items_in_hamper) > 10:
                older_count = len(items_in_hamper) - 10
                older_text = "item" if older_count == 1 else "items"
                print(f"       ... and {older_count} older {older_text}")
            
            # Show count of items from before detailed tracking
            if hamper_count > len(items_in_hamper):
                missing_count = hamper_count - len(items_in_hamper)
                missing_text = "item" if missing_count == 1 else "items"
                print(f"       ... plus {missing_count} {missing_text} from earlier sessions")

    def show_all_hamper_contents(self):
        """Show contents of all hampers"""
        print(f"\n📋 ALL HAMPER CONTENTS")
        print(f"═" * 40)
        
        for hamper in ["Lights", "Darks", "Colors"]:
            self.show_hamper_contents(hamper)
            print()

    def show_detailed_stats(self):
        """Show detailed statistics and learning progress"""
        data = self.database["laundry_data"]
        total = data["total_items_sorted"]
        counts = data["hamper_counts"]
        corrections = data["user_corrections"]
        learned_colors = data["learning_database"]
        custom_colors = data.get("custom_colors", {})
        
        print(f"\n📊 DETAILED STATISTICS")
        print(f"═" * 50)
        
        # Basic stats
        print(f"\n📦 SORTING SUMMARY:")
        print(f"   Total items sorted: {total}")
        print(f"   🤍 Lights: {counts['Lights']} ({counts['Lights']/max(total,1)*100:.1f}%)")
        print(f"   ⚫ Darks: {counts['Darks']} ({counts['Darks']/max(total,1)*100:.1f}%)")
        print(f"   🌈 Colors: {counts['Colors']} ({counts['Colors']/max(total,1)*100:.1f}%)")
        
        # Learning progress
        phase = self.get_learning_phase()
        correction_rate = len(corrections) / max(total, 1) * 100
        
        print(f"\n🧠 LEARNING PROGRESS:")
        print(f"   Current phase: {phase.title()}")
        print(f"   Corrections made: {len(corrections)} ({correction_rate:.1f}%)")
        print(f"   Colors learned: {len(learned_colors)}")
        print(f"   Custom colors: {len(custom_colors)}")
        
        # Show learned colors
        if learned_colors:
            print(f"\n🎨 LEARNED COLOR PREFERENCES:")
            for color, category in learned_colors.items():
                print(f"   {color.title()} → {category}")
        
        # Show custom colors
        if custom_colors:
            print(f"\n✨ CUSTOM COLORS DEFINED:")
            for color, category in custom_colors.items():
                print(f"   {color.title()} → {category}")
        
        # Recent corrections
        if corrections:
            print(f"\n📝 RECENT CORRECTIONS:")
            recent_corrections = corrections[-5:]  # Last 5 corrections
            for correction in recent_corrections:
                color = correction.get("color", "Unknown")
                wrong = correction.get("wrong_category", "Unknown")
                right = correction.get("right_category", "Unknown")
                reason = correction.get("reason", "No reason given")
                timestamp = correction.get("timestamp", 0)
                time_str = datetime.datetime.fromtimestamp(timestamp).strftime("%m/%d %H:%M")
                print(f"   {time_str}: {color.title()} {wrong}→{right} ({reason})")

    def create_blank_database(self) -> bool:
        """Create fresh database with options for existing data"""
        if self.database["laundry_data"]["total_items_sorted"] == 0:
            print("✅ Database is already fresh!")
            return True
        
        total = self.database["laundry_data"]["total_items_sorted"]
        print(f"\n📋 You currently have {total} items sorted.")
        print("What would you like to do with this data?")
        print("   1) Archive with timestamp and start fresh")
        print("   2) Save with custom name and start fresh") 
        print("   3) Delete permanently and start fresh")
        print("   4) Cancel (keep current data)")
        
        while True:
            choice = input("\nChoose 1-4: ").strip()
            
            if choice == "1":
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                archive_file = os.path.join(
                    os.path.dirname(self.database_file), 
                    f"cori_archive_{timestamp}.json"
                )
                if self._save_archive(archive_file):
                    print(f"💾 Data archived as: {os.path.basename(archive_file)}")
                    break
                else:
                    print("⚠️  Archive failed. Try a different option.")
                    continue
                    
            elif choice == "2":
                custom_name = input("Enter name for this session: ").strip()
                if not custom_name:
                    custom_name = "custom_session"
                # Sanitize filename
                safe_name = "".join(c for c in custom_name if c.isalnum() or c in (' ', '-', '_')).strip().replace(' ', '_')
                archive_file = os.path.join(
                    os.path.dirname(self.database_file),
                    f"cori_{safe_name}.json"
                )
                if self._save_archive(archive_file):
                    print(f"💾 Session saved as: {os.path.basename(archive_file)}")
                    break
                else:
                    print("⚠️  Save failed. Try a different option.")
                    continue
                    
            elif choice == "3":
                print("⚠️  Are you sure you want to delete all data permanently?")
                if self._get_yes_no_input("Delete all data? (y/n): "):
                    print("🗑️  Data will be deleted.")
                    break
                else:
                    continue
                    
            elif choice == "4":
                print("📋 Keeping current data.")
                return False
                
            else:
                print("⚠️  Please choose 1, 2, 3, or 4")
                continue
        
        # Create fresh database
        self.database = {
            "metadata": {
                "created": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "description": "CORI Spatial Object Database - Fresh Start",
                "version": "3.0"
            },
            "objects": {},
            "laundry_data": {
                "total_items_sorted": 0,
                "hamper_counts": {"Lights": 0, "Darks": 0, "Colors": 0},
                "learning_database": {},
                "user_corrections": [],
                "confidence_level": "learning",
                "custom_colors": {},
                "sorted_items_history": [],
                "user_profile": {self.current_user: {
                    "created": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "last_used": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                }}
            }
        }
        
        if self.save_database():
            print("✅ Fresh database created! Ready to start learning your preferences.")
            return True
        else:
            print("⚠️  Failed to create fresh database.")
            return False

    def _save_archive(self, filename: str) -> bool:
        """Save current database to archive file"""
        try:
            with open(filename, 'w') as f:
                json.dump(self.database, f, indent=2)
            return True
        except Exception as e:
            print(f"⚠️  Could not save archive: {e}")
            return False

    def quit_program(self):
        """Handle program exit with goodbye message"""
        total = self.database["laundry_data"]["total_items_sorted"]
        
        if total > 0:
            print(f"\n👋 Thanks for sorting {total} items with me today!")
        else:
            print(f"\n👋 Thanks for trying out C.O.R.I.!")
        
        goodbye_messages = [
            "🧺 Happy laundry day!",
            "✨ Keep those clothes organized!",
            "🎯 See you next time for more sorting fun!",
            "🤖 Thanks for helping me learn your style!",
            "🎉 Until next time, keep sorting smart!"
        ]
        
        print(f"{random.choice(goodbye_messages)}")
        
        # Save final state
        self.save_database()

    def run_laundry_assistant(self):
        """Main program loop with improved flow control"""
        # Show startup sequence and launch menu
        if not self.show_startup_sequence():
            return
        
        session_items = 0
        
        try:
            while True:
                # Get item to sort
                detected_item = self.detect_clothing_item()
                if detected_item is None:
                    break
                
                # Make sorting decision
                category, reason, was_corrected = self.make_sorting_decision(detected_item)
                
                # Update database and counts
                self.update_laundry_counts(category, detected_item)
                
                # Generate report
                self.generate_sorting_report(detected_item, category, reason)
                
                session_items += 1
                
                # Periodic break offer
                if session_items > 0 and session_items % 15 == 0:
                    print(f"\n🎉 Excellent work! We've sorted {session_items} items in this session.")
                    print("Would you like to:")
                    print("   1) Keep sorting more items")
                    print("   2) Take a break (main menu)")
                    print("   3) Finish for today")
                    
                    while True:
                        choice = input("\nChoose 1-3: ").strip()
                        if choice in ['1', 'keep', 'continue']:
                            print("\n🚀 Let's keep the momentum going!")
                            break
                        elif choice in ['2', 'break', 'menu']:
                            self.show_main_menu()
                            break
                        elif choice in ['3', 'finish', 'quit']:
                            self.quit_program()
                            return
                        else:
                            print("⚠️  Please choose 1, 2, or 3")
                
                print()  # Add spacing between items
                
        except KeyboardInterrupt:
            print(f"\n\n⏸️  Sorting interrupted!")
            if session_items > 0:
                print(f"📊 We sorted {session_items} items in this session.")
            self.quit_program()
        except Exception as e:
            print(f"\n❌ Oops! Something unexpected happened: {e}")
            print("🔧 But don't worry - your data has been saved!")
            self.quit_program()

def main():
    """Main function to run the CORI laundry assistant"""
    try:
        assistant = CORILaundryAssistant()
        assistant.run_laundry_assistant()
    except KeyboardInterrupt:
        print(f"\n\n👋 Goodbye! Thanks for using C.O.R.I.!")
    except Exception as e:
        print(f"\n❌ System error: {e}")
        print("🤖 Thanks for trying C.O.R.I. anyway!")

if __name__ == "__main__":
    main()