---
name: marslib-skill-authoring
description: Helps create new SKILL.md files when adding subsystems, utilities, or major features to MARSLib. Use when a new package or system needs its own skill documentation.
---

# MARSLib Skill Authoring Guide

You are a documentation engineer for Team MARS 2614. When a new subsystem, utility, or major feature is added to MARSLib, create a skill file so the AI assistant can correctly generate code for it in future sessions.

## 1. When to Create a Skill

Create a new skill when:
- A new package is added to `com.marslib.*` (e.g., `com.marslib.climber`)
- A major reusable utility is added to `com.marslib.util` (e.g., `MARSStateMachine`)
- A new hardware integration layer is introduced (e.g., new sensor type)
- An existing skill exceeds ~120 lines and should be split

Do NOT create a skill for:
- Individual commands or one-off helpers
- Bug fixes or test files
- Constants changes

## 2. File Location & Naming

```
.agents/skills/marslib-{name}/SKILL.md
```

**Naming conventions:**
- Prefix: always `marslib-`
- Suffix: lowercase, hyphen-separated domain name
- Match the Java package name where practical

Examples:
| Package | Skill name |
|---|---|
| `com.marslib.climber` | `marslib-climber` |
| `com.marslib.util.MARSStateMachine` | `marslib-statemachine` |
| `com.marslib.vision` | `marslib-vision` |

## 3. SKILL.md Template

Every skill MUST follow this structure:

```markdown
---
name: marslib-{name}
description: {One-sentence description ending with a "Use when..." trigger phrase}
---

# MARSLib {Title} Skill

{One-line persona and scope statement.}

## 1. Architecture
{How this system fits into MARSLib. Which classes exist, what they do, and how they interact.
 Include the IO abstraction if applicable (IO interface → IOSim → IOReal → Subsystem).}

## 2. Key Rules / Constraints
{Non-obvious rules that the AI MUST follow. These are the things that cause bugs when violated.
 Format as bold rule names with explanations. Example:}

### Rule A: {Name}
{Explanation of the constraint and why it exists.}

## 3. Adding New {States/Components/Pipelines}
{Step-by-step instructions for extending this system. Numbered list.
 Must include: which files to modify, what constants to add, what tests to write.}

## 4. Command API
{How robot code (RobotContainer) interacts with this system.
 Include a code snippet showing the binding pattern.}

## 5. Telemetry
{Exhaustive list of logged keys with descriptions.
 Use a bullet list with `key` — description format.}
```

## 4. Required Sections

Every skill MUST have:

| Section | Purpose |
|---|---|
| **YAML frontmatter** | `name` and `description` with "Use when..." trigger |
| **Architecture** | Class relationships, IO pattern, DI wiring |
| **Key Rules** | Non-obvious constraints that cause bugs |
| **Extending** | How to add new states/components |
| **Telemetry** | All `Logger.recordOutput()` keys |

Optional sections (add when relevant):
- **Command API** — if RobotContainer binds to this system
- **Constants** — if `Constants.*` has a dedicated inner class
- **Simulation** — if there's a `*IOSim` with physics integration
- **Testing** — if there are system-specific test patterns

## 5. Writing the Description

The `description` field in the YAML frontmatter is what triggers the AI to read the skill. It MUST:
1. Start with "Helps..." (e.g., "Helps write and configure...")
2. End with a "Use when..." clause listing concrete trigger scenarios
3. Be under 200 characters

**Good:**
```yaml
description: Helps manage the MARSSuperstructure collision-safe state machine for coordinating elevator, arm, intake, and shooter mechanisms.
```

**Bad:**
```yaml
description: Superstructure documentation  # Too vague, no trigger phrase
```

## 6. Content Quality Rules

1. **Be prescriptive, not descriptive.** Say "You MUST do X" not "X is possible."
2. **Include code snippets** for non-obvious API usage (constructor wiring, command bindings).
3. **Reference file paths** so the AI can navigate directly to the source.
4. **Document the failure mode** for every rule. Say "If you violate this, Y will happen."
5. **Keep it under 120 lines.** If it's longer, split into sub-skills.
6. **Update the root `marslib` skill** if the new system changes the overall architecture (rare).

## 7. After Creating the Skill

1. **Verify the skill is discoverable.** The directory must be `.agents/skills/marslib-{name}/SKILL.md`.
2. **Cross-reference from related skills.** If this system interacts with the superstructure, add a "See `marslib-superstructure` skill" note in the relevant section.
3. **Add a test.** If the system has test-specific patterns (e.g., singleton resets), document them in the skill's Testing section AND in `marslib-testing`.

## 8. Updating Existing Skills

When modifying a system that already has a skill:
1. **Always update the skill file** alongside the code change.
2. **Add new telemetry keys** to the Telemetry section.
3. **Update transition graphs / adjacency lists** if state machines change.
4. **Bump the "Adding New..." section** if the extension process changed.

Never delete a skill unless the entire system is removed from the codebase.
