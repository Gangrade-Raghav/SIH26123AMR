# Source Register

## Antigravity official documentation

1. Antigravity Best Practices
https://antigravity.google/docs/cli/best-practices/

Key points used in this repository:
- explore → plan → execute
- verification loops
- workspace `AGENTS.md` / `GEMINI.md`
- structured permissions
- parallel subagents
- `/rewind`, `/fork`

2. Antigravity IDE Overview
https://antigravity.google/docs/ide/overview/

Key points:
- agentic IDE
- asynchronous agents
- browser/terminal/editor operation
- artifacts and transparency

3. Antigravity Rules
https://www.antigravity.google/docs/ide/rules/

Key points:
- `.agents/rules`
- global/workspace rules
- rule activation modes

4. Antigravity Skills
https://www.antigravity.google/docs/ide/skills/

Key points:
- `.agents/skills/<skill>/SKILL.md`
- progressive disclosure
- focused reusable skills

5. Antigravity Settings
https://www.antigravity.google/docs/ide/settings/

Key points:
- command execution permissions
- workspace isolation
- artifact review
- sandboxing

## AMR architecture source

The finalized project architecture is based on the audited document:

Advanced Decentralized AMR Fleet Coordination Architecture.docx

The audit identified the following as core candidates:
- ROS 2 Jazzy
- Zenoh
- CBBA/ACBBA
- RHCR
- PIBT
- WFG deadlock reasoning
- compute-aware fallback
- controlled fault injection

The audit also identified the following as optional/deferred:
- TLC-CBBA
- GD-RHCR as an emerging research candidate
- NH-ORCA
- MAPF-LNS2
- heterogeneous PIBT

The project must preserve the distinction between established literature and the proposed system integration.
