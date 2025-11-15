# Basics of Mobile Robotics 2025 Project 
Repository for the basics of mobile robotics project.

Group n°14

Members:
- Tim Kobler
- Amir Lahlou
- Ana Schwabedal
- Jules Villaret

## How to use git
> [!NOTE]
> **See file attached in the folder "files" for more info**

Basic Idea:
- **Each person works on their designated branch**
    1. Switch to your branch (filter, vision, ...)
    2. "git pull origin main" to sync the latest version of the main branch 

- **When done with your code (for the moment):**
    1. git add .
    2. git commit -m "info about what you did since the last commit"
    3. git push origin your_working_brach_name
    4. merge the branch with the main branch if necessary (good to do regularily to avoid issues in the end when everybody merges their code). To do this, go to the githb website, and it should say that your branch is X-commits ahead of branch main, and Y-commits later from branch main. It sould offer to merge the 2 branches, while checking automatically for conflicts and compatibility issues. (Normally, we should not have compatibility issues since everyone will work on their own module.)
    5. git pull origin main to keep your branch up to date with the latest main version (you could also pull from your branch if you have not merged)

> [!WARNING]
> Never work on the main branch directly! 

Be careful about trying to pull/push from the wrong branch, and always keep in mind which version of the code you are working with.

## Usefull Links
- Docs for TDM client: https://github.com/epfl-mobots/tdm-python/blob/main/README.md

## Additionnal remarks
- None for now

## Issues
- [ ] None

