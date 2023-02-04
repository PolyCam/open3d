# open3d

## How to make changes to open3d & test them in the polycam repo:
1. Make a new branch in the open3d repo: https://github.com/PolyCam/open3d
2. Make a new branch in the vcpkg repo: https://github.com/PolyCam/vcpkg
3. Clone both repos locally on your machine & checkout your new branches in each repo
4. Make the changes you want in your local open3d repo -> commit & push the changes
5. In github, go into your open3d branch -> click on the 7 digit ID (something like 60d552f) -> copy the commit reference (something long like 00d552f22e77ce3f6a57f0eeb6cbae48e57f5066) to your clipboard
6. In your local vcpkg repo, update the ports/open3d/portfile.cmake file: paste your commit reference into line 5, which starts with “REF” -> commit & push the changes
7. To test out your change: in your Polycam repo, go to poly/cpp/vcpkg -> checkout your vcpkg branch
8. Rebuild the project from scratch (if in CLion: Tools -> CMake -> Reset Cache and Reload Project)
9. When there is an error building -> copy the “Actual hash” from the error code and paste it into ports/open3d/portfile.cmake file: line 6, which starts with “SHA512 -> commit & push the changes -> pull these in the polycam repo
10. For each new update in the open3d repo, you need to redo steps 4-9
  * Update REF in the vcpkg repo
  * Git pull in polycam repo
  * "Reset Cache and Reload Project" in polycam repo
  * Update SHA512 using "Actual hash" in error output
  * Git pull in polycam repo
  * "Reset Cache and Reload Project" in polycam repo
11. If it hits a bug, open the .log file listed in the error code -> search for “error”
