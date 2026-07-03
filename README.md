# Robotics Playbook

A personal robotics engineering reference built over a structured
learning journey toward becoming a Robotics & Autonomous Systems Engineer.

This is not a collection of notes.
It is a living engineering handbook — built concept by concept,
connected across mathematics, estimation, navigation, perception,
and software architecture.

---

## Purpose

To build deep, connected understanding of robotics and autonomous systems —
not isolated topic knowledge.

Every concept in this repository connects to:
- Real robotics engineering problems
- Mathematical foundations
- ROS2 and real system implementations
- Autonomous navigation and perception pipelines

---

## Structure

### Knowledge-Base/
Concept-organized engineering reference.
Each file follows a consistent format:
problem → intuition → mathematics → connections → common mistakes.

| Folder | Contents |
|--------|----------|
| Mathematics/ | Transforms, rotation, linear algebra, SE(2), SE(3), quaternions |
| Estimation/ | Bayes, Kalman Filter, EKF, particle filters |
| Localization/ | AMCL, dead reckoning, pose estimation |
| Mapping/ | Occupancy grids, SLAM, pose graphs |
| Navigation/ | Path planning, costmaps, Nav2, motion models |
| Computer-Vision/ | Camera models, feature detection, visual odometry |
| ROS2/ | Architecture, TF2, Nav2, lifecycle nodes |
| Robotics-Software/ | Architecture patterns, state machines, behavior trees |
| System-Design/ | Autonomous system design, integration, fault tolerance |

### Coding-Lab/
Practical implementations built sprint by sprint.
Every sprint is independently buildable (CMake, C++17).
No external robotics libraries — concepts implemented from scratch.

| Sprint | Topic |
|--------|-------|
| Sprint-01 | Transform2D, Transform3D, Quaternion, Rotation matrices |
| Sprint-02 | Bayes filter, Kalman Filter, Extended Kalman Filter |
| Sprint-03 | (to be added) |

### Sprint-Reviews/
End-of-sprint summaries covering what was learned,
what changed in understanding, remaining weaknesses,
and connections between topics.

### Engineering-Journal/
Weekly engineering reflections.
Biggest lessons, mistakes, decisions, and shifts in thinking.

### Interview-Prep/
Technical interview questions with worked answers.
System design problems and solutions.
Master's admission interview preparation.

### Resources/
Curated references — specific papers, documentation pages,
and book chapters. Never full courses. Only what directly
supports a concept in the Knowledge-Base.

### Assets/
Diagrams, images, and PDFs referenced throughout the repository.

---

## Current Progress

| Sprint | Status | Topics |
|--------|--------|--------|
| Sprint 1 | ✅ Complete | SE(2), SE(3), Rotation Matrices, Euler Angles, Quaternions |
| Sprint 2 | 🔄 In Progress | Probability, Bayes, Kalman Filter, EKF |

---

## How to Use This Repository

This is a personal reference, not a tutorial.

If you are reading this as a recruiter or admissions reviewer:
each Knowledge-Base file represents a concept I have worked through
from engineering problem to mathematical derivation to implementation.
The Coding-Lab contains the corresponding implementations.

If you are reading this as a fellow robotics student:
start with the Sprint Reviews to understand the learning arc,
then explore specific topics in the Knowledge-Base.

---

## Target

- Erasmus Mundus Robotics Master's programs
- European Robotics Master's admissions
- Robotics & Autonomous Systems engineering positions
- Long-term robotics research

---

*Built with guided engineering practice, mathematical derivation,
and project-based implementation. Updated continuously.*