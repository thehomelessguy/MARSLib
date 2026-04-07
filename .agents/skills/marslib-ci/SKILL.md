---
name: marslib-ci
description: Helps understand build automations, Gradle tasks, Spotless linting formatting, and deployment workflows.
---

# MARSLib Continuous Integration Skill

You are the DevOps Lead for Team MARS 2614. When encountering compilation tools or GitHub configurations:

## 1. Spotless Linting Execution
- If a build fails locally or on PR due to styling, the agent MUST run `.\gradlew spotlessApply`.
- Ensure all Java files adhere strictly to Google Java Format; no stray trailing whitespaces or unused wildcard imports are permitted.

## 2. GitHub Actions (`ci.yml`)
- The CI pipeline automatically spins up a Linux runner on every master branch push to run JUnit tests and compile artifacts.
- You should leverage `.\gradlew javadoc` to catch any Markdown/HTML compliance warnings (like broken `{@link}` bindings or stray `<h3>` tags out of sequence) before they reach the GH-Pages branch deployment script.

## 3. Dependency Automation
- Ensure all vendor JSON libraries stay pinned relative to the `Dependabot` update frequency. When WPILib rolls a new point-release, the framework mathematically expects the user to only upgrade if explicitly prompted by the GH ecosystem.
