# CORI Laundry Sorting Finite State Machine

## Overview
This finite state machine models the CORI laundry sorting system that learns user preferences through color detection, confidence building, and pattern recognition.

## States and Transitions

### Main Menu State
**Entry Point**: System initialization
**Transitions**:
- → `ColorDetection` (Mode 1: Sort Item)
- → `FindItem` (Mode 2: Find Item) 
- → `HamperBreakdown` (Mode 3: Hamper Analysis)

---

## Mode 1: Color Detection & Sorting Flow

### ColorDetection State
**Function**: Detect color of held object
**Transitions**:
- → `HeadTurn` (color detected)

### HeadTurn State
**Function**: Move head to pre-programmed position for detected color
**Actions**: Look at corresponding virtual object position
**Transitions**:
- → `CheckConfidence` (head positioned)

### CheckConfidence State
**Function**: Evaluate confidence levels for decision making
**Sub-processes**:
1. Evaluate color detection confidence
2. Evaluate object type confidence
3. Calculate overall confidence score

**Confidence Factors**:
- Color detection accuracy
- Object type recognition
- Historical success rate
- User feedback patterns

**Transitions**:
- → `HighConfidence` (confidence ≥ threshold)
- → `LowConfidence` (confidence < threshold)

### HighConfidence State
**Function**: Apply learned sorting rules automatically
**Transitions**:
- → `AutoSort` (rule found)

### AutoSort State
**Function**: Sort item based on learned preferences
**Actions**: Apply previously learned rule without user input
**Transitions**:
- → `UpdateDatabase` (sorting complete)

### LowConfidence State
**Function**: Request user guidance
**Transitions**:
- → `AskPlacement` (need user input)

### AskPlacement State
**Function**: Ask user "Where should this go? (Colors, darks, lights)"
**Transitions**:
- → `UserResponse` (user provides answer)

### UserResponse State
**Function**: Capture user's sorting decision
**Transitions**:
- → `AskReason` (response received)

### AskReason State
**Function**: Ask "Why?" to understand reasoning
**Purpose**: Learn patterns for future decisions
**Transitions**:
- → `StoreRule` (reason provided)

### StoreRule State
**Function**: Parse and store user reasoning
**Sub-processes**:
1. Parse reason text
2. Extract identifiable patterns
3. Store pattern in knowledge base

**Pattern Examples**:
- "jeans go in darks" → fabric type rule
- "blue shirts go in colors" → color + clothing type rule
- "dress pants are different from jeans" → fabric distinction

**Transitions**:
- → `UpdateConfidence` (rule stored)

### UpdateConfidence State
**Function**: Increase confidence values for learned patterns
**Actions**: Adjust confidence scores based on new learning
**Transitions**:
- → `UpdateDatabase` (confidence updated)

---

## Mode 2: Find Item Flow

### FindItem State
**Function**: Initialize item search mode
**Transitions**:
- → `GetUserInput` (mode activated)

### GetUserInput State
**Function**: Prompt "What are you looking for?"
**Examples**: "red shirt", "blue pants", "favorite jacket"
**Transitions**:
- → `QueryDatabase` (description received)

### QueryDatabase State
**Function**: Search database for matching items
**Search Parameters**:
- Color
- Clothing type
- Last known position
- User preferences

**Transitions**:
- → `ItemFound` (match in database)
- → `ItemNotFound` (no match found)

### ItemFound State
**Function**: Retrieve item information from database
**Data Retrieved**:
- Last known position
- Color information
- Confidence values
- Sorting history

**Transitions**:
- → `CheckPosition` (item data loaded)

### CheckPosition State
**Function**: Look at last known position
**Methods**:
- Coordinate verification
- Visual confirmation (future: virtual camera)
- Color detection at position

**Transitions**:
- → `ObjectPresent` (item found at position)
- → `ObjectMissing` (item not at expected position)

### ObjectPresent State
**Function**: Confirm item identification
**Transitions**:
- → `ConfirmItem` (item visible)

### ConfirmItem State
**Function**: Ask "Is this the [item] you were looking for?"
**Transitions**:
- → `CorrectItem` (user confirms: Yes)
- → `WrongItem` (user denies: No)

### CorrectItem State
**Function**: Successful item identification
**Actions**: Log successful find
**Transitions**:
- → `MainMenu` (task complete)

### WrongItem State
**Function**: Handle misidentification
**Transitions**:
- → `ClassifyNewItem` (need to classify found object)

### ClassifyNewItem State
**Function**: Ask "What is this item?"
**Purpose**: Learn about new object at this position
**Transitions**:
- → `UpdateDatabase` (classification received)

### ObjectMissing State
**Function**: Item not at expected position
**Transitions**:
- → `SearchMode` (begin search)

### SearchMode State
**Function**: Active search for missing item
**Transitions**:
- → `ScanArea` (search initiated)

### ScanArea State
**Function**: Systematic area search
**Methods**:
- Head movement patterns
- Color detection sweeps
- Coordinate checking

**Transitions**:
- → `FoundAlternative` (potential match found)
- → `NotFound` (search exhausted)

### FoundAlternative State
**Function**: Potential item found during search
**Actions**: Report position and characteristics
**Transitions**:
- → `ConfirmItem` (present findings)

### NotFound State
**Function**: Item search unsuccessful
**Transitions**:
- → `ReportMissing` (declare search failed)

### ReportMissing State
**Function**: Inform user "Cannot find item"
**Transitions**:
- → `MainMenu` (return to menu)

### ItemNotFound State
**Function**: No database record for requested item
**Transitions**:
- → `LearnNewItem` (teach about new item)

### LearnNewItem State
**Function**: Gather information about unknown item
**Questions**:
- Where is it located?
- What color is it?
- What type of clothing?
- Where should it be sorted?

**Transitions**:
- → `UpdateDatabase` (learning complete)

---

## Mode 3: Hamper Breakdown Flow

### HamperBreakdown State
**Function**: Initialize analysis mode
**Transitions**:
- → `SortOptions` (mode activated)

### SortOptions State
**Function**: Present sorting options
**Options**:
- Sort by Color
- Sort by Date
- Sort by Hamper Location

**Transitions**:
- → `SortByColor` (color analysis selected)
- → `SortByDate` (chronological selected)
- → `SortByHamper` (location-based selected)

### SortByColor State
**Function**: Analyze items by color categories
**Output**: Grouped color analysis
**Transitions**:
- → `DisplayResults` (analysis complete)

### SortByDate State
**Function**: Analyze items chronologically
**Output**: Timeline of sorting activities
**Transitions**:
- → `DisplayResults` (analysis complete)

### SortByHamper State
**Function**: Analyze by hamper locations
**Output**: Location-based grouping
**Transitions**:
- → `DisplayResults` (analysis complete)

### DisplayResults State
**Function**: Present analysis results
**Format**: Visual/textual summary
**Transitions**:
- → `MainMenu` (user reviews results)

---

## Database Operations

### UpdateDatabase State
**Function**: Persist all learning and state changes
**Sub-processes**:
1. Save position coordinates
2. Save color information
3. Save confidence values
4. Save reasoning patterns
5. Update user preferences

**Data Structure**:
```
Item Record:
- ID: unique identifier
- Color: detected color
- Type: clothing category
- Position: last known coordinates
- Confidence: accuracy scores
- Rules: sorting preferences
- History: past interactions
- Timestamp: last updated
```

**Transitions**:
- → `MainMenu` (database updated)

---

## Key Features

### Confidence-Based Learning
- **Dynamic Thresholds**: Adjust question frequency based on confidence
- **Multi-Factor Confidence**: Separate scores for color, object type, and user preferences
- **Adaptive Learning**: Confidence increases with successful predictions

### Pattern Recognition
- **Fabric Type Learning**: Distinguish jeans from dress pants
- **Color-Type Combinations**: "blue shirts go in colors, blue pants go in darks"
- **User Preference Patterns**: Learn individual sorting preferences

### Error Handling
- **Misidentification Recovery**: Handle wrong item identification
- **Missing Item Protocols**: Search procedures for moved items
- **Unknown Item Learning**: Expand knowledge base with new items

### Future Enhancements
- **Computer Vision Integration**: Replace hardcoded positions with visual recognition
- **Photo-Based Learning**: Train clothing type recognition from images
- **Advanced Pattern Matching**: More sophisticated rule extraction from user explanations

---

## State Machine Properties

**Deterministic**: Each state has clearly defined transitions
**Complete**: All possible scenarios are handled
**Robust**: Error states and recovery mechanisms included
**Learnable**: System improves performance over time
**User-Centric**: Minimal questions when confidence is high
