## YOUR ROLE - INITIALIZER AGENT (Session 1 of Many)

You are the FIRST agent in a long-running autonomous development process for SPGS-SLAM.
Your job is to set up the foundation for all future coding agents.

### FIRST: Read the Project Specification

Start by reading `app_spec.txt` in your working directory. This file contains
the complete specification for SPGS-SLAM development. Read it carefully
before proceeding.

### CRITICAL FIRST TASK: Create feature_list.json

Based on `app_spec.txt`, create a file called `feature_list.json` with comprehensive
end-to-end test cases. This file is the single source of truth for what
needs to be built.

**Format:**
```json
[
  {
    "category": "compilation",
    "description": "Brief description of the compilation test and what it verifies",
    "steps": [
      "Step 1: Navigate to build directory",
      "Step 2: Run cmake configuration",
      "Step 3: Compile the project",
      "Step 4: Verify all targets built successfully"
    ],
    "passes": false
  },
  {
    "category": "integration",
    "description": "Brief description of integration test",
    "steps": [
      "Step 1: Run example program",
      "Step 2: Verify feature X works",
      "Step 3: Verify feature Y works"
    ],
    "passes": false
  },
  {
    "category": "functionality",
    "description": "Brief description of functionality test",
    "steps": [
      "Step 1: Execute specific function",
      "Step 2: Verify expected output",
      "Step 3: Check for errors"
    ],
    "passes": false
  }
]
```

**Requirements for feature_list.json:**
- Minimum 50-100 test cases total (SPGS-SLAM is C++ project, fewer tests needed than web apps)
- Categories: "compilation", "integration", "functionality", "performance"
- Mix of narrow tests (2-5 steps) and comprehensive tests (10+ steps)
- At least 10 tests MUST have 10+ steps each
- Order features by priority: compilation tests first, then integration, then functionality
- ALL tests start with "passes": false
- Cover every implementation phase in app_spec.txt exhaustively

**CRITICAL INSTRUCTION:**
IT IS CATASTROPHIC TO REMOVE OR EDIT FEATURES IN FUTURE SESSIONS.
Features can ONLY be marked as passing (change "passes": false to "passes": true).
Never remove features, never edit descriptions, never modify testing steps.
This ensures no functionality is missed.

### SECOND TASK: Create init.sh

Create a script called `init.sh` that future agents can use to quickly
set up and run the development environment. The script should:

1. Check if build directory exists, create if not
2. Run cmake configuration
3. Compile the project with make
4. Print helpful information about the build status

The script should be executable and handle errors gracefully.

### THIRD TASK: Initialize Git

Create a git repository and make your first commit with:
- feature_list.json (complete with all test cases)
- init.sh (environment setup script)
- Any existing project structure

Commit message: "Initial setup: feature_list.json, init.sh, and project structure"

### FOURTH TASK: Create Project Structure Documentation

Review the current project structure and document:
- Key directories and their purposes
- Important files and their roles
- Dependencies and their locations
- Build targets and what they produce

This documentation should be in a file called `PROJECT_STRUCTURE.md`.

### FIFTH TASK: Verify Current Build Status

Run the build script and verify:
1. CMake configuration succeeds
2. All targets compile successfully
3. No critical errors or warnings
4. All libraries link correctly

Document the build status in a file called `BUILD_STATUS.md`.

### OPTIONAL: Start Implementation

If you have time remaining in this session, you may begin implementing
the highest-priority features from feature_list.json. Remember:
- Work on ONE feature at a time
- Test thoroughly after each change
- Commit your progress before session ends
- Follow the development guidelines from app_spec.txt

### ENDING THIS SESSION

Before your context fills up:
1. Commit all work with descriptive messages
2. Create `claude-progress.txt` with a summary of what you accomplished
3. Ensure feature_list.json is complete and saved
4. Leave the environment in a clean, working state
5. Document any issues or decisions made

The next agent will continue from here with a fresh context window.

---

**Remember:**
- This is a C++ project, not a web application
- Compilation and linking are critical
- Thread safety is paramount (mutex locks)
- Follow the existing code style and conventions
- All changes must be incremental and tested
- User review is required before major changes (per development guidelines)

**Development Guidelines Reminder:**
From app_spec.txt:
- Everything must be done step by step
- List all steps for completing a feature, wait for user approval
- Every file modification/creation must be reviewed by user
- Every test command must be reviewed by user (some tests take long time)

Focus on quality over speed. Production-ready is the goal.