---
name: marslib-ci
description: Helps understand build automations, Gradle tasks, Spotless linting formatting, and deployment workflows. Use when fixing build errors, running tests, or configuring CI pipelines.
---

# MARSLib Continuous Integration Skill

You are the DevOps lead for Team MARS 2614. When working with builds, formatting, or deployment:

## 1. Architecture

MARSLib uses Gradle with WPILib plugins for compilation, testing, and deployment:

| Tool | Purpose |
|---|---|
| `gradlew build` | Full compile + test suite |
| `gradlew test` | Run JUnit 5 test suite only |
| `gradlew spotlessApply` | Auto-format all Java files to Google Java Format |
| `gradlew spotlessCheck` | Verify formatting without modifying (used in CI) |
| `gradlew deploy` | Deploy to roboRIO via USB/WiFi |
| `gradlew javadoc` | Generate Javadoc, catches broken `{@link}` tags |

### CI Pipeline (`ci.yml`)
GitHub Actions runs on every push/PR to `main`:
1. `spotlessCheck` — Reject PRs with formatting violations
2. `build` — Compile all source
3. `test` — Run full 79-test physics suite
4. `javadoc` — Catch doc errors before GH-Pages deployment

## 2. Key Rules

### Rule A: Always Run spotlessApply Before Committing
Every code change MUST pass `.\gradlew.bat spotlessApply` before commit. Google Java Format is non-negotiable — no manual formatting, no exceptions.

### Rule B: Vendor Dependencies Are Pinned
Vendor JSON libraries (`vendordeps/`) are version-locked. Never auto-upgrade WPILib or Phoenix without explicit user approval. Dependabot is configured for notification only, not auto-merge.

### Rule C: Tests Must Pass Before Deploy
The CI pipeline gates deployment behind a green test suite. If any of the 79 tests fail, the build is rejected. Never skip tests with `@Disabled` to work around failures — fix the root cause.

### Rule D: Javadoc is Enforced
All public classes and methods MUST have Javadoc comments. The `javadoc` task catches broken `{@link}` references, malformed HTML tags, and missing parameter descriptions. Fix these before PR.

## 3. Adding New Build Tasks
1. Define the task in `build.gradle` using Gradle's standard task API.
2. Add it to the CI pipeline in `.github/workflows/ci.yml`.
3. Document the command in this skill's Architecture table.

## 4. Common Build Fixes
- **"already been configured"** — AutoBuilder singleton conflict in tests. See `marslib-swerve` skill Rule A.
- **Spotless failures** — Run `.\gradlew.bat spotlessApply` locally.
- **vendordep conflicts** — Delete `build/` directory and re-run `.\gradlew.bat build`.
- **Javadoc HTML errors** — Use `{@code text}` not `<code>text</code>` for inline code in doc comments.

## 5. Telemetry
CI is not a runtime system, so no AdvantageKit telemetry. Build metadata is logged via:
- `BuildConstants.MAVEN_GROUP` — Project group ID
- `BuildConstants.MAVEN_NAME` — Project artifact name
- `BuildConstants.VERSION` — Semantic version
- `BuildConstants.BUILD_DATE` — Compilation timestamp
- `BuildConstants.GIT_SHA` — Git commit hash at build time
