#!/usr/bin/env python3
"""
CORI Laundry Sorting Assistant
Intelligent laundry sorting with learning capabilities and personality
"""

import json
import time
import random
import datetime
import os
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass

@dataclass
class DetectedItem:
    color: str
    confidence: float
    timestamp: float

class CORILaundryAssistant:
    def __init__(self, database_file: str = "../../../shared/shared/shared/database/cori_spatial_database.json"):
        self.database_file = database_file
        self.database = self.load_database()
        self.initialize_laundry_system()
        
    def initialize_laundry_system(self):
        """Initialize laundry-specific database structure"""
        if "laundry_data" not in self.database:
            self.database["laundry_data"] = {
                "total_items_sorted": 0,
                "hamper_counts": {"Lights": 0, "Darks": 0, "Colors": 0},
                "learning_database": {},  # color -> category mappings
                "user_corrections": [],
                "confidence_level": "learning",  # learning -> tentative -> confident
                "custom_colors": {},  # user-defined colors
                "sorted_items_history": []  # track what went where
            }
            self.save_database()
    
    def load_database(self) -> Dict:
        """Load the CORI spatial database"""
        try:
            with open(self.database_file, 'r') as f:
                database = json.load(f)
                # Ensure laundry_data exists with all required keys
                if "laundry_data" not in database or not isinstance(database["laundry_data"], dict):
                    database["laundry_data"] = {
                        "total_items_sorted": 0,
                        "hamper_counts": {"Lights": 0, "Darks": 0, "Colors": 0},
                        "learning_database": {},
                        "user_corrections": [],
                        "confidence_level": "learning",
                        "custom_colors": {},
                        "sorted_items_history": []
                    }
                return database
        except FileNotFoundError:
            print(f"Creating new database file: {self.database_file}")
            return {
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
                    "sorted_items_history": []
                }
            }
    
    def save_database(self):
        """Save the database back to file"""
        try:
            with open(self.database_file, 'w') as f:
                json.dump(self.database, f, indent=2)
        except Exception as e:
            print(f"Warning: Could not save database: {e}")
    
    def get_learning_phase(self) -> str:
        """Determine CORI's current learning phase"""
        total_sorted = self.database["laundry_data"]["total_items_sorted"]
        
        if total_sorted < 15:
            return "learning"
        elif total_sorted < 30:
            return "tentative"
        else:
            return "confident"
    
    def show_startup_sequence(self):
        """Show cool CORI startup sequence with improved ASCII art"""
        print("\n" * 2)
        
        # Animated startup effect
        print("🤖 Initializing C.O.R.I. system", end="")
        for i in range(4):
            time.sleep(0.4)
            print(".", end="", flush=True)
        print("\n")
        time.sleep(0.3)
        
        # Improved C.O.R.I. ASCII art with better formatting
        print("╔" + "═" * 68 + "╗")
        print("║" + " " * 68 + "║")
        print("║" + "   ██████╗    ██████╗    ██████╗    ██╗".center(68) + "║")
        print("║" + "  ██╔════╝   ██╔═══██╗   ██╔══██╗   ██║".center(68) + "║")
        print("║" + "  ██║        ██║   ██║   ██████╔╝   ██║".center(68) + "║")
        print("║" + "  ██║        ██║   ██║   ██╔══██╗   ██║".center(68) + "║")
        print("║" + "  ╚██████╗██╗╚██████╔╝██╗██║  ██║██╗██║".center(68) + "║")
        print("║" + "   ╚═════╝╚═╝ ╚═════╝ ╚═╝╚═╝  ╚═╝╚═╝╚═╝".center(68) + "║")
        print("║" + " " * 68 + "║")
        print("║" + "    Cooperative Organizational Robotic Intelligence".center(68) + "║")
        print("║" + "                 Laundry Sorting Assistant".center(68) + "║")
        print("║" + " " * 68 + "║")
        
        # Ensure laundry_data exists before accessing
        if "laundry_data" not in self.database or not isinstance(self.database["laundry_data"], dict):
            self.initialize_laundry_system()
        
        # Show current status
        learning_phase = self.get_learning_phase()
        total_sorted = self.database["laundry_data"]["total_items_sorted"]
        
        status_line = f"Status: {learning_phase.title()} Mode | Items Sorted: {total_sorted}"
        print("║" + status_line.center(68) + "║")
        print("║" + " " * 68 + "║")
        print("╚" + "═" * 68 + "╝")
        
        # Add some personality based on learning phase
        if learning_phase == "learning":
            print("\n🎓 I'm still learning! Please help me by correcting any mistakes.")
        elif learning_phase == "tentative":
            print("\n🤔 I'm getting more confident, but I might still need guidance.")
        else:
            print("\n💪 I'm feeling confident about my sorting abilities!")
        
        print("\n" + "─" * 50)
        print("Ready to sort your laundry! 🧺✨")
        print("─" * 50 + "\n")
        
        time.sleep(0.5)
        
        # Check if there's existing data
        if total_sorted > 0:
            items_text = "item" if total_sorted == 1 else "items"
            counts = self.database["laundry_data"]["hamper_counts"]
            print(f"I found existing data! You've already sorted {total_sorted} {items_text}.")
            print(f"Current hampers: Lights({counts['Lights']}) | Darks({counts['Darks']}) | Colors({counts['Colors']})")
            
            print("\nWhat would you like to do?")
            print("   1) 🔄 Continue where we left off")
            print("   2) 👀 Check what's in the hampers")
            print("   3) 🆕 Start fresh (archive current data)")
            print("   4) 🚪 Exit C.O.R.I.")
            
            while True:
                choice = input(f"\n> What's it gonna be? (1-4): ").strip()
                
                if choice in ['1', 'continue', 'sort']:
                    print("\n🚀 Awesome! Let's continue sorting your laundry!")
                    return self.detect_clothing_item()
                elif choice in ['2', 'check', 'hampers']:
                    self.show_all_hamper_contents()
                    input("\nPress enter to continue...")
                    return self.show_startup_sequence()
                elif choice in ['3', 'fresh', 'new']:
                    success = self.create_blank_database()
                    if success:
                        print("\n🎉 Fresh start activated! Let's begin sorting!")
                        return self.detect_clothing_item()
                    else:
                        return self.show_startup_sequence()
                elif choice in ['4', 'quit', 'exit']:
                    return None
                else:
                    print("❌ Please choose 1, 2, 3, or 4 (or say 'continue', 'check', 'fresh', 'quit')")
        else:
            print("Looks like this is your first time! Welcome aboard! 🎉")
            print("\nWhat would you like to do?")
            print("   1) 🆕 Start sorting laundry!")
            print("   2) 📖 Learn more about how C.O.R.I. works")
            print("   3) 🚪 Exit for now")
            
            while True:
                choice = input(f"\n> Ready to dive in? (1-3): ").strip()
                
                if choice in ['1', 'start', 'sort']:
                    print("\n🎉 Perfect! Let's create your laundry sorting profile!")
                    print("I'll learn your preferences as we go. This is going to be fun!")
                    return self.detect_clothing_item()
                elif choice in ['2', 'learn', 'help']:
                    self.show_cori_explanation()
                    return self.show_startup_sequence()
                elif choice in ['3', 'quit', 'exit']:
                    print("\n👋 No problem! Come back anytime!")
                    return None
                else:
                    print("❌ Please choose 1, 2, or 3 (or say 'start', 'learn', 'quit')")
    
    def show_cori_explanation(self):
        """Explain how CORI works in detail"""
        print(f"\n📖 HOW C.O.R.I. WORKS:")
        print(f"=" * 50)
        print(f"\n🎯 THREE LEARNING PHASES:")
        print(f"   Phase 1 (Items 1-15): I ask you about everything")
        print(f"   Phase 2 (Items 16-30): I make suggestions, you approve")
        print(f"   Phase 3 (Items 31+): I sort confidently, you can override")
        
        print(f"\n🗂️  THREE HAMPER CATEGORIES:")
        print(f"   🤍 LIGHTS: Whites, creams, light colors")
        print(f"   ⚫ DARKS: Blacks, navy, dark colors that might bleed")
        print(f"   🌈 COLORS: Bright colors, medium tones")
        
        print(f"\n🧠 SMART FEATURES:")
        print(f"   • I remember your preferences for each color")
        print(f"   • I learn new colors you teach me")
        print(f"   • I adapt to YOUR specific sorting style")
        print(f"   • I get better every time we work together")
        
        input(f"\nPress enter to continue...")
    
    def create_blank_database(self):
        """Create a completely fresh database with user choice for old data"""
        # Check if current database has data
        if self.database["laundry_data"]["total_items_sorted"] > 0:
            print(f"\n📋 You currently have {self.database['laundry_data']['total_items_sorted']} items sorted.")
            print("What would you like to do with this data?")
            print("   1) Save it with a custom name for later")
            print("   2) Auto-archive with timestamp") 
            print("   3) Delete it (permanent)")
            print("   4) Cancel (keep current data)")
            
            choice = input("\nChoose 1, 2, 3, or 4: ").strip()
            
            if choice == "1":
                # Custom name save
                custom_name = input("Enter a name for this sorting session: ").strip()
                if not custom_name:
                    custom_name = "custom_session"
                
                # Clean the name for filename
                safe_name = "".join(c for c in custom_name if c.isalnum() or c in (' ', '-', '_')).rstrip()
                safe_name = safe_name.replace(' ', '_')
                
                archive_file = self.database_file.replace(".json", f"_{safe_name}.json")
                
                try:
                    with open(archive_file, 'w') as f:
                        json.dump(self.database, f, indent=2)
                    print(f"💾 Session saved as: {archive_file}")
                except Exception as e:
                    print(f"⚠️  Could not save session: {e}")
                    
            elif choice == "2":
                # Auto-archive with timestamp
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                archive_file = self.database_file.replace(".json", f"_archive_{timestamp}.json")
                
                try:
                    with open(archive_file, 'w') as f:
                        json.dump(self.database, f, indent=2)
                    print(f"💾 Session archived as: {archive_file}")
                except Exception as e:
                    print(f"⚠️  Could not archive session: {e}")
                    
            elif choice == "3":
                # Delete - just confirm
                confirm = input("Are you sure you want to delete this data? (y/n): ").strip().lower()
                if confirm not in ['y', 'yes']:
                    print("📋 Keeping current data. Canceling fresh start.")
                    return False
                else:
                    print("🗑️  Previous data will be deleted")
            elif choice == "4":
                print("📋 Keeping current data. Canceling fresh start.")
                return False
            else:
                print("📋 Invalid choice. Keeping current data.")
                return False
        
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
                "sorted_items_history": []
            }
        }
        self.save_database()
        print("✅ Fresh database created! Ready to start learning your preferences.")
        return True
    
    def detect_clothing_item(self) -> Optional[DetectedItem]:
        """Simulate clothing item detection with natural conversation"""
        total_sorted = self.database["laundry_data"]["total_items_sorted"]
        
        # Natural, varied prompts based on progress
        prompts = [
            "What's next?",
            "Show me another item!",
            "What do you have for me?",
            "Ready for the next one!",
            "Let's keep going - what's this?",
            "Bring on the next piece!",
            "What are we sorting now?",
            "Show me what you've got!",
            "Another item coming up?",
            "What's this one?"
        ]
        
        # Occasional encouragement based on progress
        if total_sorted > 0 and total_sorted % 5 == 0:
            encouragements = [
                f"Nice work! We've sorted {total_sorted} items together.",
                f"You're getting the hang of this! {total_sorted} items done.",
                f"Great teamwork! {total_sorted} items sorted so far.",
                f"Look at us go! {total_sorted} items in the books."
            ]
            print(f"\n🎉 {random.choice(encouragements)}")
        
        # Simple, natural prompt
        if total_sorted == 0:
            prompt = "What's the first item?"
        else:
            prompt = random.choice(prompts)
            
        print(f"\n{prompt}")
        print("(Examples: 'red shirt', 'blue jeans' | Type 'help', 'stats', 'menu', or 'quit')")
        
        item_input = input("> ").strip().lower()
        
        if item_input in ['quit', 'exit']:
            return None
        elif item_input == 'menu':
            return self.show_main_menu()
        elif item_input in ['help', '?']:
            print("\n💡 Just tell me what you see! Like 'red shirt' or 'blue jeans'")
            print("   Type 'menu' for options, 'quit' to exit, 'stats' for progress")
            return self.detect_clothing_item()
        elif item_input == 'stats':
            self.show_quick_stats()
            return self.detect_clothing_item()
        
        return self.process_item_input(item_input)
    
    def show_quick_stats(self):
        """Show quick stats without the full menu"""
        counts = self.database["laundry_data"]["hamper_counts"]
        total = self.database["laundry_data"]["total_items_sorted"]
        phase = self.get_learning_phase()
        
        items_text = "item" if total == 1 else "items"
        print(f"\n📊 Quick stats: {total} {items_text} sorted")
        print(f"   Lights: {counts['Lights']} | Darks: {counts['Darks']} | Colors({counts['Colors']})")
        
        if phase == "learning":
            remaining = 15 - total
            print(f"   I'm still learning! {remaining} more until I get confident.")
        elif phase == "tentative":
            remaining = 30 - total  
            print(f"   Getting smarter! {remaining} more until I'm an expert.")
        else:
            print(f"   I'm a laundry sorting expert now! 🎓")
    
    def show_main_menu(self) -> Optional[DetectedItem]:
        """Show the main menu when explicitly requested"""
        total_sorted = self.database["laundry_data"]["total_items_sorted"]
        counts = self.database["laundry_data"]["hamper_counts"]
        
        items_text = "item" if total_sorted == 1 else "items"
        
        print(f"\n🤖 Hey! I've sorted {total_sorted} {items_text} and I'm getting better at understanding your preferences.")
        print(f"\nCurrent hampers:  Lights({counts['Lights']}) | Darks({counts['Darks']}) | Colors({counts['Colors']})")
        
        # Show custom colors if any
        if "custom_colors" in self.database["laundry_data"] and self.database["laundry_data"]["custom_colors"]:
            custom_colors = list(self.database["laundry_data"]["custom_colors"].keys())
            print(f"Custom colors I know: {', '.join(custom_colors)}")
        
        print("\nWhat would you like to do?")
        print("   1) Continue sorting items")
        print("   2) Check hamper contents ('lights', 'darks', 'colors', or 'all')")
        print("   3) Start fresh training session")
        print("   4) Quit")
        
        user_input = input("\n> ").strip().lower()
        
        if user_input in ['1', 'continue', 'sort', '']:
            return self.detect_clothing_item()
        elif user_input in ['3', 'new', 'reset', 'fresh', 'start over']:
            success = self.create_blank_database()
            if success:
                return self.detect_clothing_item()
            else:
                return self.show_main_menu()
        elif user_input in ['4', 'quit', 'exit']:
            return None
        elif user_input in ['2', 'lights', 'darks', 'colors', 'all']:
            if user_input in ['lights', 'darks', 'colors']:
                self.show_hamper_contents(user_input.title())
            else:
                self.show_all_hamper_contents()
            input("\nPress enter to continue...")
            return self.show_main_menu()
        else:
            # They might be entering an item directly
            return self.process_item_input(user_input)
    
    def process_item_input(self, item_input: str) -> Optional[DetectedItem]:
        """Process the item input from user with natural responses"""
        # Parse the combined color + clothing input
        color_input, clothing_type = self.parse_item_input(item_input)
        
        if not color_input:
            responses = [
                "Hmm, I didn't catch that. Try 'red shirt' or 'blue pants'",
                "Could you be more specific? Like 'green sweater'?",
                "I need a color and item type - like 'white socks'",
                "What color is it? And what kind of clothing?"
            ]
            print(f"🤔 {random.choice(responses)}")
            return self.detect_clothing_item()
        
        # Normalize color input and handle common variations/misspellings
        color_input = self.normalize_color_input(color_input)
        
        # Get current known colors from database
        known_colors = self.get_known_colors()
        
        if color_input not in known_colors:
            # New color detected - ask user to define it naturally
            new_color_category = self.define_new_color(color_input)
            if not new_color_category:
                return self.detect_clothing_item()  # User cancelled, try again
        
        # More natural detection responses
        detection_responses = [
            f"Ah, a {color_input} {clothing_type}!",
            f"I see a {color_input} {clothing_type}.",
            f"Got it - {color_input} {clothing_type}!",
            f"Nice, a {color_input} {clothing_type}.",
            f"Okay, {color_input} {clothing_type} coming up!"
        ]
        
        print(f"👀 {random.choice(detection_responses)}")
        
        # Brief pause for more natural feel
        time.sleep(0.5)
        
        return DetectedItem(
            color=f"{color_input}#{clothing_type}",
            confidence=random.uniform(0.85, 0.98),
            timestamp=time.time()
        )
    
    def parse_item_input(self, item_input: str) -> Tuple[Optional[str], str]:
        """Parse combined color + clothing input like 'green pants' or 'white shirt'"""
        if not item_input or item_input == 'quit':
            return None, ""
        
        # Common clothing types to look for
        clothing_types = [
            'shirt', 'shirts', 'pants', 'jeans', 'shorts', 'sock', 'socks', 
            'dress', 'skirt', 'jacket', 'coat', 'sweater', 'hoodie', 'top',
            'blouse', 'tank', 'tee', 't-shirt', 'polo', 'cardigan', 'vest',
            'underwear', 'bra', 'boxers', 'briefs', 'pajamas', 'pjs', 'sleepwear',
            'towel', 'sheets', 'pillowcase', 'blanket'
        ]
        
        words = item_input.split()
        
        # Look for clothing type in the input
        clothing_type = "item"  # default
        color_words = []
        
        for word in words:
            if word in clothing_types:
                clothing_type = word
            else:
                color_words.append(word)
        
        # Reconstruct color from remaining words
        color_input = " ".join(color_words).strip()
        
        # Handle common patterns
        if not color_input and len(words) >= 2:
            # Try assuming first word is color, rest is clothing type
            color_input = words[0]
            clothing_type = " ".join(words[1:])
        elif not color_input:
            # Single word input - assume it's a color
            color_input = item_input
            clothing_type = "item"
        
        return color_input if color_input else None, clothing_type
    
    def normalize_color_input(self, color_input: str) -> str:
        """Normalize color input and handle common variations/misspellings"""
        # Handle common misspellings and variations
        color_corrections = {
            # Grey/Gray variations
            'light_grey': 'light grey', 'light_gray': 'light grey', 'lightgrey': 'light grey', 'lightgray': 'light grey',
            'dark_grey': 'dark grey', 'dark_gray': 'dark grey', 'darkgrey': 'dark grey', 'darkgray': 'dark grey',
            
            # Pink variations
            'light_pink': 'light pink', 'lightpink': 'light pink',
            'bright_pink': 'bright pink', 'brightpink': 'bright pink',
            
            # Blue variations
            'dark_blue': 'dark blue', 'darkblue': 'dark blue',
            'bright_blue': 'bright blue', 'brightblue': 'bright blue',
            
            # Green variations
            'dark_green': 'dark green', 'darkgreen': 'dark green',
            'bright_green': 'bright green', 'brightgreen': 'bright green',
            
            # Common misspellings
            'yelow': 'yellow', 'yelllow': 'yellow', 'purpel': 'purple', 'purpal': 'purple',
            'ornage': 'orange', 'oragne': 'orange', 'creme': 'cream', 'navey': 'navy', 'nafy': 'navy'
        }
        
        return color_corrections.get(color_input, color_input)
    
    def get_known_colors(self) -> List[str]:
        """Get all known colors from database and defaults"""
        # Default colors
        default_colors = [
            'white', 'light grey', 'tan', 'yellow', 'cream', 'light pink',
            'red', 'blue', 'green', 'orange', 'purple', 'bright pink', 
            'black', 'dark grey', 'navy', 'dark blue', 'brown', 'dark green'
        ]
        
        # Add any custom colors from database
        if "custom_colors" in self.database["laundry_data"]:
            custom_colors = list(self.database["laundry_data"]["custom_colors"].keys())
            default_colors.extend(custom_colors)
        
        return default_colors
    
    def define_new_color(self, color_input: str) -> Optional[str]:
        """Allow user to define a new color and its category"""
        print(f"\n🎨 I don't know the color '{color_input}' yet!")
        print("Would you like to teach me about this color?")
        
        teach = input("Teach me this color? (y/n): ").strip().lower()
        if teach not in ['y', 'yes']:
            print("No problem! Try using a color I already know.")
            return None
        
        print(f"\n🤔 Where should '{color_input}' items usually go?")
        print("1) Lights (whites, creams, light colors)")
        print("2) Darks (blacks, dark colors that might bleed)")  
        print("3) Colors (bright colors, medium tones)")
        
        while True:
            choice = input("Choose 1, 2, or 3: ").strip()
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
                print("Please choose 1, 2, or 3")
        
        # Save the new color to database
        if "custom_colors" not in self.database["laundry_data"]:
            self.database["laundry_data"]["custom_colors"] = {}
        
        self.database["laundry_data"]["custom_colors"][color_input] = category
        self.save_database()
        
        print(f"✅ Great! I'll remember that '{color_input}' goes in {category}")
        return category
    
    def get_default_category(self, color: str) -> str:
        """Get default laundry category based on color"""
        lights = ['white', 'light grey', 'tan', 'yellow', 'cream', 'light pink']
        darks = ['black', 'dark grey', 'navy', 'dark blue', 'brown', 'dark green']
        colors = ['red', 'blue', 'green', 'orange', 'purple', 'bright pink']
        
        base_color = color.split('#')[0] if '#' in color else color
        
        # Check custom colors first
        if "custom_colors" in self.database["laundry_data"]:
            if base_color in self.database["laundry_data"]["custom_colors"]:
                return self.database["laundry_data"]["custom_colors"][base_color]
        
        # Check default categories
        if base_color in lights:
            return "Lights"
        elif base_color in darks:
            return "Darks"
        elif base_color in colors:
            return "Colors"
        else:
            return "Colors"  # Default fallback
    
    def get_cori_reasoning(self, color: str, suggested_category: str) -> str:
        """Generate CORI's reasoning for sorting decision"""
        base_color = color.replace('#', ' ').split(' ')[0] if '#' in color else color.replace('_', ' ')
        
        reasoning_map = {
            "Lights": {
                "white": "White items need to stay bright and clean!",
                "light grey": "Light colors should stay with other lights to avoid getting dingy",
                "tan": "Tan is delicate and light - safer with the whites",
                "yellow": "Yellow can get muddy with dark colors",
                "cream": "Cream is basically white - definitely goes with lights",
                "light pink": "Light pink is too delicate for the other loads"
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
                "red": "Red has good color but isn't dark enough to go with darks",
                "blue": "Blue is bright but not too light or too dark",
                "green": "Green is a nice middle-ground color",
                "orange": "Orange is vibrant but not bleeding-level dark",
                "purple": "Purple is colorful but not delicate like lights",
                "bright pink": "Bright pink has personality - goes with other colors"
            }
        }
        
        # Check if it's a custom color
        if "custom_colors" in self.database["laundry_data"]:
            if base_color in self.database["laundry_data"]["custom_colors"]:
                return f"You taught me that {base_color} items go in {suggested_category}"
        
        return reasoning_map.get(suggested_category, {}).get(base_color, 
                                                           f"{base_color} seems like a good fit for {suggested_category}")
    
    def make_sorting_decision(self, item: DetectedItem) -> Tuple[str, str, bool]:
        """Make sorting decision with natural, conversational style"""
        phase = self.get_learning_phase()
        color = item.color
        base_color = color.split('#')[0] if '#' in color else color
        clothing_type = color.split('#')[1] if '#' in color else 'item'
        
        # Check if we've learned this color before
        learned_category = self.database["laundry_data"]["learning_database"].get(base_color)
        
        if phase == "learning":
            # Learning phase - casual questions
            if learned_category:
                casual_checks = [
                    f"I remember putting {base_color} stuff in {learned_category} before.",
                    f"Last {base_color} item went in {learned_category}.",
                    f"You usually put {base_color} things in {learned_category}.",
                    f"I think {base_color} items go in {learned_category}."
                ]
                print(f"🤔 {random.choice(casual_checks)}")
                print(f"Same deal for this {clothing_type}? (y/n)")
                
                confirm = input("> ").strip().lower()
                if confirm in ['y', 'yes']:
                    return learned_category, "following your pattern", False
            
            new_item_responses = [
                f"Hmm, first {base_color} {clothing_type} I've seen!",
                f"New one for me - {base_color} {clothing_type}.",
                f"Haven't sorted a {base_color} {clothing_type} before.",
                f"This {base_color} {clothing_type} is new to me."
            ]
            print(f"🤷‍♂️ {random.choice(new_item_responses)}")
            print("Where should it go?")
            print("   1) Lights")
            print("   2) Darks") 
            print("   3) Colors")
            
            while True:
                user_choice = input("> ").strip()
                if user_choice == "1" or user_choice.lower() == "lights":
                    return "Lights", "you taught me", True
                elif user_choice == "2" or user_choice.lower() == "darks":
                    return "Darks", "you taught me", True
                elif user_choice == "3" or user_choice.lower() == "colors":
                    return "Colors", "you taught me", True
                else:
                    print("Choose 1 (Lights), 2 (Darks), or 3 (Colors)")
                
        elif phase == "tentative":
            # Tentative phase - confident suggestions
            if learned_category:
                suggestion_phrases = [
                    f"Based on your style, this probably goes in {learned_category}.",
                    f"You usually put {base_color} items in {learned_category}.",
                    f"Following your pattern, I'd say {learned_category}.",
                    f"Looks like a {learned_category} item to me."
                ]
                print(f"🤖 {random.choice(suggestion_phrases)}")
            else:
                suggested = self.get_default_category(color)
                reasoning = self.get_cori_reasoning(color, suggested)
                print(f"🤖 I'm thinking {suggested}.")
                print(f"💭 {reasoning}")
                learned_category = suggested
            
            confirm_phrases = ["Sound right?", "Good?", "Agree?", "Make sense?"]
            print(f"{random.choice(confirm_phrases)} (y/n)")
            confirm = input("> ").strip().lower()
            if confirm in ['y', 'yes']:
                return learned_category, "following my suggestion", False
            else:
                print("🤔 Okay, where would you put it?")
                print("   1) Lights")
                print("   2) Darks")
                print("   3) Colors")
                while True:
                    user_choice = input("> ").strip()
                    if user_choice == "1" or user_choice.lower() == "lights":
                        correction_reason = input("Why there? ").strip()
                        self.record_correction(base_color, learned_category, "Lights", correction_reason)
                        return "Lights", f"you corrected me: {correction_reason}", True
                    elif user_choice == "2" or user_choice.lower() == "darks":
                        correction_reason = input("Why there? ").strip()
                        self.record_correction(base_color, learned_category, "Darks", correction_reason)
                        return "Darks", f"you corrected me: {correction_reason}", True
                    elif user_choice == "3" or user_choice.lower() == "colors":
                        correction_reason = input("Why there? ").strip()
                        self.record_correction(base_color, learned_category, "Colors", correction_reason)
                        return "Colors", f"you corrected me: {correction_reason}", True
                    else:
                        print("Choose 1 (Lights), 2 (Darks), or 3 (Colors)")
        
        else:  # confident phase
            # Confident phase - make decisions with style
            if learned_category:
                confident_statements = [
                    f"Easy - {learned_category}!",
                    f"This goes in {learned_category} for sure.",
                    f"Definitely {learned_category}.",
                    f"{learned_category}, no question."
                ]
                print(f"✅ {random.choice(confident_statements)}")
                print(f"💭 That's where all your {base_color} items go.")
                suggested = learned_category
                reason = "following your established pattern"
            else:
                suggested = self.get_default_category(color)
                reasoning = self.get_cori_reasoning(color, suggested)
                confident_statements = [
                    f"I'm putting this in {suggested}.",
                    f"This belongs in {suggested}.",
                    f"{suggested} is the right choice.",
                    f"Going with {suggested} on this one."
                ]
                print(f"✅ {random.choice(confident_statements)}")
                print(f"💭 {reasoning}")
                reason = reasoning
            
            override_checks = ["Different idea?", "Change of plans?", "Override?"]
            print(f"{random.choice(override_checks)} (y/n)")
            override = input("> ").strip().lower()
            if override in ['y', 'yes']:
                print("Where instead?")
                print("   1) Lights")
                print("   2) Darks")
                print("   3) Colors")
                while True:
                    user_choice = input("> ").strip()
                    if user_choice == "1" or user_choice.lower() == "lights":
                        correction_reason = input("Why there? ").strip()
                        self.record_correction(base_color, suggested, "Lights", correction_reason)
                        return "Lights", f"you corrected me: {correction_reason}", True
                    elif user_choice == "2" or user_choice.lower() == "darks":
                        correction_reason = input("Why there? ").strip()
                        self.record_correction(base_color, suggested, "Darks", correction_reason)
                        return "Darks", f"you corrected me: {correction_reason}", True
                    elif user_choice == "3" or user_choice.lower() == "colors":
                        correction_reason = input("Why there? ").strip()
                        self.record_correction(base_color, suggested, "Colors", correction_reason)
                        return "Colors", f"you corrected me: {correction_reason}", True
                    else:
                        print("Choose 1 (Lights), 2 (Darks), or 3 (Colors)")
            else:
                return suggested, reason, False
    
    def record_correction(self, color: str, wrong_category: str, right_category: str, reason: str):
        """Record when user corrects CORI's decision with natural feedback"""
        correction = {
            "color": color,
            "wrong_category": wrong_category,
            "right_category": right_category,
            "reason": reason,
            "timestamp": time.time()
        }
        
        self.database["laundry_data"]["user_corrections"].append(correction)
        self.database["laundry_data"]["learning_database"][color] = right_category
        
        # More natural responses
        responses = [
            f"Ah, got it! {color.title()} items go in {right_category}. I'll remember that.",
            f"Thanks for the correction! {color.title()} → {right_category}. Makes sense!",
            f"Good to know! I'll put {color} items in {right_category} from now on.",
            f"Perfect! {color.title()} goes in {right_category}. Learning something new!"
        ]
        
        print(f"📝 {random.choice(responses)}")
    
    def update_laundry_counts(self, category: str, item: DetectedItem):
        """Update hamper counts and learning database"""
        base_color = item.color.split('#')[0] if '#' in item.color else item.color
        item_name = item.color.replace('#', ' ').title()
        
        # Update counts
        self.database["laundry_data"]["hamper_counts"][category] += 1
        self.database["laundry_data"]["total_items_sorted"] += 1
        
        # Update learning database
        self.database["laundry_data"]["learning_database"][base_color] = category
        
        # Track sorted items history for hamper contents
        if "sorted_items_history" not in self.database["laundry_data"]:
            self.database["laundry_data"]["sorted_items_history"] = []
        
        item_record = {
            "item_name": item_name,
            "category": category,
            "timestamp": item.timestamp,
            "color": base_color
        }
        
        self.database["laundry_data"]["sorted_items_history"].append(item_record)
        
        self.save_database()
    
    def generate_sorting_report(self, item: DetectedItem, category: str, reason: str):
        """Generate natural sorting report"""
        clothing_name = item.color.replace('#', ' ').title()
        
        # Natural completion messages
        completion_messages = [
            f"Done! The {clothing_name} is now in the {category} hamper.",
            f"Perfect! {clothing_name} → {category} hamper.",
            f"All set! Your {clothing_name} is sorted into {category}.",
            f"There we go! {clothing_name} goes in {category}."
        ]
        
        print(f"\n{random.choice(completion_messages)}")
        if reason and not reason.startswith("following"):
            print(f"💡 {reason}")
        
        # Show updated counts more naturally
        counts = self.database["laundry_data"]["hamper_counts"]
        total = sum(counts.values())
        
        print(f"\n📊 Current hampers: Lights({counts['Lights']}) | Darks({counts['Darks']}) | Colors({counts['Colors']})")
        
        # Show learning progress naturally
        phase = self.get_learning_phase()
        if phase == "learning" and total < 15:
            remaining_to_learn = 15 - total
            remaining_text = "item" if remaining_to_learn == 1 else "items"
            print(f"🧠 Still learning your style! ({remaining_to_learn} more {remaining_text} until I get more confident)")
        elif phase == "tentative" and total < 30:
            remaining_to_confident = 30 - total
            remaining_text = "item" if remaining_to_confident == 1 else "items"
            print(f"🤖 Getting the hang of this! ({remaining_to_confident} more {remaining_text} until I'm fully confident)")
        elif total == 15:
            print(f"🎉 I'm starting to understand your preferences! Moving to suggestion mode.")
        elif total == 30:
            print(f"🎓 I've got your laundry style down! I'll make confident decisions now.")
    
    def show_hamper_contents(self, hamper_name: str):
        """Show contents of a specific hamper"""
        # Get all items from the sorting history that went to this hamper
        items_in_hamper = []
        
        if "sorted_items_history" not in self.database["laundry_data"]:
            self.database["laundry_data"]["sorted_items_history"] = []
        
        for item_record in self.database["laundry_data"]["sorted_items_history"]:
            if item_record.get("category") == hamper_name:
                items_in_hamper.append(item_record)
        
        print(f"\n📦 {hamper_name.upper()} HAMPER:")
        
        # Check the hamper count vs history
        hamper_count = self.database["laundry_data"]["hamper_counts"].get(hamper_name, 0)
        
        if hamper_count == 0:
            print(f"   Empty - no {hamper_name.lower()} items sorted yet")
        elif not items_in_hamper and hamper_count > 0:
            items_text = "item" if hamper_count == 1 else "items"
            print(f"   {hamper_count} {items_text} (sorted before detailed tracking was enabled)")
            print(f"   Start sorting new items to see detailed history!")
        else:
            items_text = "item" if len(items_in_hamper) == 1 else "items"
            print(f"   {len(items_in_hamper)} {items_text}:")
            for i, item in enumerate(items_in_hamper[-10:], 1):  # Show last 10 items
                item_name = item.get("item_name", "Unknown item")
                timestamp = item.get("timestamp", 0)
                # Convert timestamp to readable format
                time_str = datetime.datetime.fromtimestamp(timestamp).strftime("%m/%d %H:%M")
                print(f"   {i}. {item_name} (sorted {time_str})")
            
            if len(items_in_hamper) > 10:
                remaining_items = len(items_in_hamper) - 10
                remaining_text = "item" if remaining_items == 1 else "items"
                print(f"   ... and {remaining_items} more {remaining_text}")
            
            # Show if there are items not in detailed history
            if hamper_count > len(items_in_hamper):
                missing_items = hamper_count - len(items_in_hamper)
                missing_text = "item" if missing_items == 1 else "items"
                print(f"   ... plus {missing_items} {missing_text} from before detailed tracking")
    
    def show_all_hamper_contents(self):
        """Show contents of all hampers"""
        print(f"\n📋 ALL HAMPER CONTENTS:")
        
        for hamper in ["Lights", "Darks", "Colors"]:
            self.show_hamper_contents(hamper)
            print()  # Add spacing between hampers
    
    def run_laundry_assistant(self):
        """Execute the complete CORI laundry sorting workflow with cool startup"""
        # Show the awesome startup sequence first
        detected_item = self.show_startup_sequence()
        
        if detected_item is None:
            goodbye_messages = [
                "👋 Thanks for checking out C.O.R.I.! See you later!",
                "🤖 Goodbye! Come back anytime you need help sorting!",
                "✨ Catch you later! Keep those clothes organized!",
                "🧺 See ya! C.O.R.I. will be here when you need me!"
            ]
            print(f"\n{random.choice(goodbye_messages)}")
            return
        
        items_since_last_break = 0
        
        while True:
            # If we don't have a detected item, get one
            if detected_item is None:
                detected_item = self.detect_clothing_item()
                
            if detected_item is None:
                print("\n👋 Thanks for using C.O.R.I.! See you next time!")
                break
            
            # Step 2: Sorting Decision
            category, reason, was_corrected = self.make_sorting_decision(detected_item)
            
            # Step 3: Update Database
            self.update_laundry_counts(category, detected_item)
            
            # Step 4: Generate Report
            self.generate_sorting_report(detected_item, category, reason)
            
            # Step 5: Track items since last break
            items_since_last_break += 1
            
            # Reset detected_item for next iteration
            detected_item = None
            
            # Ask if user wants to continue - every 10 items or when they want to stop
            total_sorted = self.database["laundry_data"]["total_items_sorted"]
            
            if items_since_last_break >= 10:
                # Every 10 items, ask if they want to keep going
                items_text = "item" if items_since_last_break == 1 else "items"
                print(f"\n🎉 Great job! We've sorted {items_since_last_break} {items_text} in this session.")
                print("Want to keep going with more items?")
                print("   1) Yes, keep sorting!")
                print("   2) Take a break (back to menu)")
                print("   3) Quit for now")
                
                continue_choice = input("> ").strip()
                if continue_choice in ['1', 'yes', 'y', 'keep going']:
                    items_since_last_break = 0  # Reset counter
                    print("\n🚀 Let's keep sorting!")
                elif continue_choice in ['2', 'menu', 'break']:
                    # Return to main menu instead of ending
                    session_text = "item" if items_since_last_break == 1 else "items"
                    print(f"\n📊 Session summary: {items_since_last_break} {session_text} sorted!")
                    items_since_last_break = 0  # Reset counter
                    detected_item = self.show_main_menu()  # Go back to main menu
                    if detected_item is None:
                        break
                else:
                    print("\n👋 Thanks for sorting with C.O.R.I. today!")
                    break
            else:
                # For the first few items, don't ask every time - just continue
                print()  # Just add a space and continue

def main():
    """Main function to run the CORI laundry assistant"""
    assistant = CORILaundryAssistant()
    
    try:
        assistant.run_laundry_assistant()
    except KeyboardInterrupt:
        # Natural goodbye messages instead of robotic interruption message
        goodbye_messages = [
            "\n\n👋 Okay, see ya! Thanks for sorting with me!",
            "\n\n🙌 Goodbye! Catch you later for more laundry fun!",
            "\n\n👋 See you next time! Happy laundry day!",
            "\n\n🧺 Bye! Thanks for teaching me your sorting style!",
            "\n\n✨ Later! Keep those clothes organized!"
        ]
        print(f"{random.choice(goodbye_messages)}")
    except Exception as e:
        print(f"\n❌ Oops, something went wrong: {e}")
        print("👋 But hey, thanks for using C.O.R.I. anyway!")

if __name__ == "__main__":
    main()