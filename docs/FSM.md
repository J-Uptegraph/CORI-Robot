# CORI Learning Manipulation FSM
## Adaptive Color Sorting with Human Teaching

> *CORI learns by doing, remembers by asking, and grows with every user interaction*

**Core Philosophy:** CORI begins with no knowledge of color sorting. Through human guidance and reinforcement, it develops a personalized understanding of your preferred color sorting system.

---

## System Overview

- **Baseline Goal:** Scan real-world colors, manipulate virtual objects, and perform autonomous pick-and-place tasks.  
- **Reach Goal:** Learn sorting rules entirely through human interaction, without pre-programmed logic.  
- **Stretch Goal:** Enable collaborative decision-making with memory and confirmation loops.

---

## Finite State Machine

| State             | Description                                  | Human Interaction                              | Learning Opportunity                  | Transition            |
|-------------------|----------------------------------------------|-----------------------------------------------|---------------------------------------|-----------------------|
| `IDLE`            | Awaits real-world color detection            | Present colored object to webcam              | Tracks usage patterns                 | Color detected        |
| `LOCATE_OBJECT`   | Searches table for matching virtual object   | None (autonomous)                             | Improves object detection accuracy    | Object found          |
| `APPROACH_TARGET` | Moves arm to hover above object              | None (autonomous)                             | Optimizes movement                    | Above object          |
| `VERIFY_GRASP`    | Confirms object with palm camera             | None (autonomous)                             | Enhances visual verification          | Object confirmed      |
| `PICK_OBJECT`     | Grasps and lifts object                      | None (autonomous)                             | Refines grasp success patterns        | Object secured        |
| `QUERY_PLACEMENT` | Asks human for sorting decision              | "Where should this [color] go?"               | Learns color-to-bin mapping           | Human responds        |
| `MOVE_TO_BIN`     | Transports object to specified bin           | None (autonomous)                             | Improves navigation efficiency        | Above target bin      |
| `PLACE_OBJECT`    | Releases object into bin                     | None (autonomous)                             | Enhances placement accuracy           | Object released       |
| `CONFIRM_CHOICE`  | Verifies learning with human                 | "[Color] → [Bin]. Remember this?"             | Reinforces preferences                | Human confirms        |
| `UPDATE_MEMORY`   | Stores or updates color preference           | None (internal)                               | Correlates patterns                   | Memory updated        |
| `READY_NEXT`      | Returns to home position, signals readiness  | "Ready for next object!"                      | Analyzes session patterns             | Returns to `IDLE`     |

---

## Learning Framework

### Memory Structure
```yaml
color_preferences:
  bright_red:
    bin: colors
    confidence: 0.95
    learned_from: 3
    last_confirmed: "2025-06-20"
  dark_blue:
    bin: darks
    confidence: 0.8
    learned_from: 2
    last_confirmed: "2025-06-19"
  light_gray:
    bin: unknown
    confidence: 0.0
    learned_from: 0
```

### Decision Logic
- **New Color:** Ask, "Where should this [color] go?"  
- **Seen 1-2 Times:** Confirm, "Last time you put [color] in [bin]. Same this time?"  
- **Seen 3+ Times (Consistent):** Place autonomously, confirm, "[Color] → [bin], right?"  
- **Human Correction:** Reset confidence, request guidance.

### Human Interaction Patterns

#### First Encounter (Unknown Color)
- **CORI:** "I've picked up this red object. Where should it go?"  
- **HUMAN:** "Colors bin"  
- **CORI:** "Red → colors bin. Got it! I'll remember that."

#### Second Encounter (Building Confidence)
- **CORI:** "Another red object. Last time you said colors bin. Same this time?"  
- **HUMAN:** "Yes"  
- **CORI:** "Perfect! Red is definitely colors bin."

#### Third+ Encounter (High Confidence)
- **CORI:** "Red object - going to colors bin, right?"  
- **HUMAN:** "Yes"  
- **CORI:** "Thought so! Getting the hang of your system."

#### Correction/Learning Moment
- **CORI:** "Red object - going to colors bin, right?"  
- **HUMAN:** "Actually, this red is more of a pink. Put it in lights."  
- **CORI:** "Ah! This shade is different. Pink-red → lights bin. I'll ask about red shades from now on."

### Progressive Learning Stages

#### Stage 1: Blank Slate (Weeks 1-2)
- **Behavior:** Queries every object.  
- **Goal:** Establishes basic color-to-bin associations.  
- **Success:** Asks relevant questions, retains answers.

#### Stage 2: Pattern Recognition (Weeks 3-4)
- **Behavior:** Confirms learned patterns, queries edge cases.  
- **Goal:** Builds confidence in common colors.  
- **Success:** 70% of objects require confirmation only.

#### Stage 3: Collaborative Sorting (Month 2+)
- **Behavior:** Autonomous for confident colors, collaborative for uncertainties.  
- **Goal:** Efficient sorting with minimal human input.  
- **Success:** 90% accuracy with rapid confirmation loops.

### State Details

#### QUERY_PLACEMENT - Learning Core
- **Purpose:** Converts unknown colors into learned preferences.  
- **Interaction Modes:**  
  - First Time: "Where should this [color] go?"  
  - Building: "Last time [color] went to [bin]. Same?"  
  - Confirming: "[Color] → [bin], right?"  
  - Uncertain: "This [color] seems different. Where should it go?"  
- **Learning Capture:**  
  - Stores HSV color ranges.  
  - Records human decisions.  
  - Tracks contextual factors (e.g., time, prior objects).  
  - Adjusts confidence based on consistency.

#### CONFIRM_CHOICE - Reinforcement Learning
- **Purpose:** Strengthens or corrects associations.  
- **Confirmation Types:**  
  - Positive Reinforcement: "Perfect! [Color] always goes to [bin]."  
  - Confidence Building: "Getting consistent with [color] → [bin]!"  
  - Pattern Recognition: "I notice all dark colors go to darks bin."  
  - Error Correction: "Noted! This [color] is actually different."

#### UPDATE_MEMORY - Knowledge Integration
- **Purpose:** Evolves understanding with new data.  
- **Memory Operations:**  
  - New Color: Adds to database.  
  - Reinforcement: Increases confidence.  
  - Correction: Adjusts mapping, lowers confidence.  
  - Pattern: Generalizes rules (e.g., "dark shades → darks").

### Success Metrics

#### Learning Velocity
- **Week 1:** 10+ colors learned with basic mappings.  
- **Month 1:** 80% of common colors sorted with confirmation.  
- **Month 3:** 95% accuracy with minimal intervention.

#### Interaction Quality
- **Appropriate Questions:** Queries only when uncertain.  
- **Memory Retention:** Consistently recalls prior decisions.  
- **Pattern Recognition:** Generalizes from examples.  
- **Error Recovery:** Gracefully handles corrections.

#### Partnership Satisfaction
- **Trust Building:** Human trusts CORI with familiar colors.  
- **Efficiency:** Sorting accelerates over time.  
- **Adaptability:** Adapts to user preferences and corrections.

---

## Implementation Architecture

### Core Components
- **Manipulation Pipeline:** Detects, grasps, and transports objects.  
- **Memory System:** Stores color preferences persistently.  
- **Dialogue Manager:** Formulates questions, processes responses.  
- **Learning Engine:** Calculates confidence, recognizes patterns.  
- **Decision Framework:** Balances autonomous actions and queries.

### Data Flow
1. Detect real-world color.  
2. Locate virtual object.  
3. Grasp object.  
4. Analyze color, check memory.  
5. Query human or confirm choice.  
6. Place object.  
7. Update learning.  
8. Prepare for next object.

### Key Technologies
- **ROS 2:** System integration and node communication.  
- **OpenCV:** Color analysis and object detection.  
- **MoveIt2:** Arm trajectory planning.  
- **JSON/SQLite:** Preference storage.  
- **Natural Language:** Simple dialogue templates.

---

## Development Phases

### Phase 1: 
- Real-to-virtual color detection.  
- Object location via head camera.  
- Object verification via palm camera.  
- Basic pick-and-place to fixed bins.

### Phase 2: 
- Memory system for color preferences.  
- Human query system.  
- Basic pattern recognition and confidence scoring.  
- Confirmation and correction handling.

### Phase 3: 
- Intelligent questioning (ask vs. confirm).  
- Pattern generalization (e.g., dark colors → darks).  
- Context awareness (e.g., time, user energy).  
- Robust error recovery.

---

## Vision Realized

### Traditional Approach
- Pre-programmed color rules.  
- Static logic, no adaptation.  
- No personalization.

### CORI's Approach
- Starts with no assumptions.  
- Learns from human interaction.  
- Builds personalized sorting system.  
- Adapts over time.

### The Result
A robot that learns *your* way of sorting, remembers your preferences, and becomes more helpful with each interaction. This FSM transforms a basic pick-and-place task into a foundation for human-robot collaboration.

> *Every question CORI asks strengthens the partnership. Every answer you give builds a smarter robot.*
