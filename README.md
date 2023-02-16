# ARSIS-6
The sixth iteration of of the ARSIS project.

## Project Summary

The Boise State NASA SUITS team is currently in the process of developing an augmented 
reality application, Augmented Reality Space Informatics System (ARSIS). The purpose of 
ARSIS is to help astronauts more easily complete EVA tasks.

Currently, real-life astronauts’ mission objectives are completed via written or 
memorized instructions. SUITS addresses this problem by creating an AR environment which 
will also communicate with ground control, and then be displayed in the astronaut’s helmet 
with up-to-date information and instructions about the task at hand. 

See last year's [ARSIS project](https://github.com/NASA-SUITS-Teams/ARSIS-6).

## Technologies

### Augmented Reality User Interface
- [Unity 3D](https://unity.com/) Real-Time 3D Engine

### Ground Control
- [ReactJS](https://reactjs.org/) Front-end Interface
- [NodeJS](https://nodejs.org/en/) Front-end Runtime Environment
- [FastAPI](https://fastapi.tiangolo.com/) Back-end API

### Telemetry
- [FastAPI](https://fastapi.tiangolo.com/) Back-end API

## Try Out The Project
See section "Contributing To The Project" for local development instructions.
1. Clone this repo with `git clone <repo-url-here>` and `cd` into the repo directory.
2. Make sure you have [Unity Hub](https://unity.com/unity-hub) installed AND Unity Editor version 2020.3.8f1.
3. Open Unity Hub and Click "Open" to open a project from your computer.
4. Navigate to and click on the ARSIS-Unity folder inside the cloned project.
5. To start the Telemetry and Ground Control servers, navigate to the ARSIS-Telemetry-GroundControl directory and run the command `docker-compose up`
6. Become the ultimate NASA SUITS member!

## Contributing To The Project
See more detailed [instructions here](https://www.tomasbeuzen.com/post/git-fork-branch-pull/)
1. From the [main project page](https://github.com/BSU-SUITS-Team/ARSIS-6) click "Fork" near the top right to create your own variation of the project.
2. Once your fork is created, click the green "Code" button on your fork's Github page and copy the HTTP link to your clipboard.
3. Open a terminal and navigate to the folder you would like your project to be in (ie, a folder called "Projects").
4. Once in the folder, use `git clone <your-fork-repo-url-here>` to create a clone of the project in that directory.
5. Add a remote repository called "upstream" pointing to the original repository with the command `git remote add upstream <original-repo-url-here>`
6. Create a new branch with the name "new_branch" with the command `git checkout -b new_feature`
7. Make your desired changes to the project and add them to a commit with `git add -A` and commit your changes with `git commit -m "Descriptive commit message here"`
8. If changes are made to the upstream project, you will want to update your fork with those changes to avoid merge conflicts. Do this with `git checkout main` and then `git pull upstream main`
9. Now that your fork has been updated, you should update your current "new_feature" branch with `git checkout new_feature` and `git merge main`
10. When you believe your feature is production ready, push your branch to your fork with `git push origin new_feature`
11. Open a pull request in Github to have your feature reviewed and hopefully merged!
