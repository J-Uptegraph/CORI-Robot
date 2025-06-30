#!/usr/bin/env python3
"""
CORI Spatial Object Database
Maps virtual laundry objects to angular positions for sensor fusion demo
"""

import json
import math
import time
import os
from typing import Dict, List, Tuple, Optional

class SpatialObjectDatabase:
    """
    Database that tracks object locations in angular coordinates relative to CORI's head
    Based on the actual positions in laundry_world.sdf
    """
    
    def __init__(self, db_file: str = "shared/shared/database/cori_spatial_database.json"):
        self.db_file = db_file
        self.objects = {}
        self.initialize_from_world_data()
        self.load_or_create_database()
    
    def initialize_from_world_data(self):
        """Initialize database from actual world object positions"""
        # CORI's position in world (assumed at origin facing +X)
        cori_position = (0.0, 0.0)  # x, y in world coordinates
        cori_facing = 0.0  # facing +X direction (0 degrees)
        
        # Object positions from laundry_world.sdf
        world_objects = {
            # Laundry items on table
            "shirt_green": {"position": (1.0, 0.0), "color": "green"},
            "sock_white": {"position": (1.2, 0.1), "color": "white"},  
            "sock_dark": {"position": (1.1, -0.1), "color": "black"},
            "shirt_red": {"position": (0.8, 0.2), "color": "red"},
            "pants_blue": {"position": (0.7, -0.2), "color": "blue"},
            "towel_gray": {"position": (1.3, -0.2), "color": "gray"},
            
            # Laundry bins (for reference)
            "bin_darks": {"position": (2.0, 0.5), "color": "black"},
            "bin_colors": {"position": (2.0, 0.0), "color": "orange"},
            "bin_lights": {"position": (2.0, -0.5), "color": "white"},
        }
        
        # Convert world positions to angular positions
        for obj_name, obj_data in world_objects.items():
            obj_x, obj_y = obj_data["position"]
            color = obj_data["color"]
            
            # Calculate angle from CORI's position to object
            dx = obj_x - cori_position[0]
            dy = obj_y - cori_position[1]
            
            # Calculate angle in degrees (0° = straight ahead, +90° = left, -90° = right)
            angle_rad = math.atan2(dy, dx)
            angle_deg = math.degrees(angle_rad)
            
            # Calculate distance for confidence scoring
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Generate confidence based on "experience" (closer objects = higher confidence)
            base_confidence = max(0.3, min(0.95, 1.0 - (distance - 0.5) * 0.2))
            
            # Add some realistic variation in success/failure counts
            success_count = max(1, int(10 * base_confidence))
            failure_count = max(0, int(10 * (1 - base_confidence)))
            
            # Store in database format
            color_key = f"{color}_objects"
            if color_key not in self.objects:
                self.objects[color_key] = []
            
            self.objects[color_key].append({
                "object_name": obj_name,
                "primary_location": round(angle_deg, 1),
                "distance": round(distance, 2),
                "confidence": round(base_confidence, 2),
                "last_seen": time.time(),
                "search_radius": round(max(10.0, 30.0 * (1 - base_confidence)), 1),
                "success_count": success_count,
                "failure_count": failure_count,
                "world_position": {"x": obj_x, "y": obj_y},
                "notes": f"Initialized from world position at ({obj_x}, {obj_y})"
            })
    
    def load_or_create_database(self):
        """Load existing database or create new one"""
        if os.path.exists(self.db_file):
            try:
                with open(self.db_file, 'r') as f:
                    loaded_data = json.load(f)
                    
                # Merge with world data (world data takes precedence for positions)
                for color_key, objects in self.objects.items():
                    if color_key in loaded_data:
                        # Keep learning data but update positions
                        for obj in objects:
                            loaded_obj = self.find_object_in_loaded_data(
                                loaded_data[color_key], obj["object_name"]
                            )
                            if loaded_obj:
                                # Keep learning stats but update position
                                obj["confidence"] = loaded_obj.get("confidence", obj["confidence"])
                                obj["success_count"] = loaded_obj.get("success_count", obj["success_count"])
                                obj["failure_count"] = loaded_obj.get("failure_count", obj["failure_count"])
                                obj["last_seen"] = loaded_obj.get("last_seen", obj["last_seen"])
                
                print(f"✅ Loaded existing database and merged with world data")
                
            except Exception as e:
                print(f"⚠️  Error loading database: {e}")
                print("📝 Creating new database from world data")
        
        self.save_database()
        self.print_database_summary()
    
    def find_object_in_loaded_data(self, loaded_objects: List, object_name: str) -> Optional[Dict]:
        """Find object in loaded data by name"""
        for obj in loaded_objects:
            if obj.get("object_name") == object_name:
                return obj
        return None
    
    def save_database(self):
        """Save database to JSON file"""
        try:
            # Add metadata
            database_with_metadata = {
                "metadata": {
                    "created": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "description": "CORI Spatial Object Database - Maps virtual objects to angular positions",
                    "coordinate_system": "Degrees from CORI center (0° = straight ahead, +° = left, -° = right)",
                    "total_objects": sum(len(objs) for objs in self.objects.values())
                },
                "objects": self.objects
            }
            
            with open(self.db_file, 'w') as f:
                json.dump(database_with_metadata, f, indent=2)
            print(f"💾 Database saved to {self.db_file}")
            
        except Exception as e:
            print(f"❌ Error saving database: {e}")
    
    def print_database_summary(self):
        """Print a nice summary of the database"""
        print("\n" + "="*80)
        print("🗃️  CORI SPATIAL OBJECT DATABASE")
        print("="*80)
        
        total_objects = 0
        for color_key, objects in self.objects.items():
            color_name = color_key.replace("_objects", "").upper()
            print(f"\n📍 {color_name} OBJECTS:")
            print("-" * 40)
            
            for obj in objects:
                total_objects += 1
                angle = obj["primary_location"]
                confidence = obj["confidence"]
                distance = obj["distance"]
                success_rate = obj["success_count"] / (obj["success_count"] + obj["failure_count"])
                
                # Direction indicator
                if angle > 15:
                    direction = "⬅️  LEFT"
                elif angle < -15:
                    direction = "➡️  RIGHT"
                else:
                    direction = "⬆️  STRAIGHT"
                
                print(f"   {obj['object_name']:<15} → {angle:>6.1f}° {direction}")
                print(f"   {'':15}   Distance: {distance:.1f}m, Confidence: {confidence:.2f}, Success: {success_rate:.1%}")
        
        print(f"\n📊 SUMMARY: {total_objects} objects tracked across {len(self.objects)} color categories")
        print("="*80 + "\n")
    
    def get_predicted_location(self, color: str) -> Optional[Dict]:
        """Get predicted location for a color (mimics the demo interface)"""
        color_key = f"{color.lower()}_objects"
        if color_key in self.objects and self.objects[color_key]:
            # Return the most confident object of this color
            best_obj = max(self.objects[color_key], key=lambda x: x["confidence"])
            return {
                'location': best_obj['primary_location'],
                'confidence': best_obj['confidence'], 
                'search_radius': best_obj['search_radius'],
                'object_name': best_obj['object_name'],
                'distance': best_obj['distance']
            }
        return None
    
    def update_success(self, color: str, found_location: float):
        """Update database when object found (mimics the demo interface)"""
        color_key = f"{color.lower()}_objects"
        if color_key in self.objects and self.objects[color_key]:
            # Find closest object to the found location
            best_match = min(
                self.objects[color_key], 
                key=lambda x: abs(x["primary_location"] - found_location)
            )
            
            best_match["success_count"] += 1
            best_match["last_seen"] = time.time()
            
            # Update confidence
            total_attempts = best_match["success_count"] + best_match["failure_count"]
            best_match["confidence"] = min(0.95, best_match["success_count"] / total_attempts)
            
            # Update search radius
            best_match["search_radius"] = max(5.0, 30.0 * (1 - best_match["confidence"]))
            
            self.save_database()
            print(f"✅ Updated {best_match['object_name']}: confidence now {best_match['confidence']:.2f}")
    
    def update_failure(self, color: str):
        """Update database when object not found"""
        color_key = f"{color.lower()}_objects"
        if color_key in self.objects and self.objects[color_key]:
            # Update the most confident object of this color
            best_obj = max(self.objects[color_key], key=lambda x: x["confidence"])
            
            best_obj["failure_count"] += 1
            
            # Update confidence
            total_attempts = best_obj["success_count"] + best_obj["failure_count"]
            best_obj["confidence"] = best_obj["success_count"] / total_attempts
            
            # Increase search radius
            best_obj["search_radius"] = min(60.0, best_obj["search_radius"] * 1.2)
            
            self.save_database()
            print(f"⚠️  Updated {best_obj['object_name']}: confidence now {best_obj['confidence']:.2f}")
    
    def list_objects_by_confidence(self) -> List[Tuple[str, float, float]]:
        """Return list of (object_name, angle, confidence) sorted by confidence"""
        all_objects = []
        for color_key, objects in self.objects.items():
            for obj in objects:
                all_objects.append((
                    obj["object_name"],
                    obj["primary_location"], 
                    obj["confidence"]
                ))
        
        return sorted(all_objects, key=lambda x: x[2], reverse=True)
    
    def get_demo_script(self) -> List[str]:
        """Generate demo script showing best objects to test"""
        demo_objects = self.list_objects_by_confidence()[:5]  # Top 5 most confident
        
        script = [
            "🎯 RECOMMENDED DEMO SEQUENCE:",
            "Hold colored objects in front of camera in this order for best results:",
            ""
        ]
        
        for i, (obj_name, angle, confidence) in enumerate(demo_objects, 1):
            color = obj_name.split('_')[1] if '_' in obj_name else obj_name
            direction = "LEFT" if angle > 15 else "RIGHT" if angle < -15 else "STRAIGHT"
            script.append(f"{i}. {color.upper()} object → CORI will look {direction} ({angle:+.1f}°) [confidence: {confidence:.1%}]")
        
        return script


def main():
    """Demo the database creation and usage"""
    print("🚀 Initializing CORI Spatial Object Database...")
    
    # Create database
    db = SpatialObjectDatabase()
    
    # Show demo script
    print("\n".join(db.get_demo_script()))
    
    # Test the interface that the sensor fusion demo will use
    print(f"\n🧪 TESTING DEMO INTERFACE:")
    print("-" * 40)
    
    test_colors = ["red", "green", "white", "blue"]
    for color in test_colors:
        result = db.get_predicted_location(color)
        if result:
            print(f"🔍 {color.upper()} → {result['location']:+.1f}° (confidence: {result['confidence']:.1%}, object: {result['object_name']})")
        else:
            print(f"❌ {color.upper()} → No objects found")
    
    print(f"\n✅ Database ready for sensor fusion demo!")
    print(f"📁 Saved to: {db.db_file}")


if __name__ == "__main__":
    main()