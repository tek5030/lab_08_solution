# Visual Odometry, proposed solution

This is our proposed solution for the lab ["Visual Odometry"][repo] in the computer vision course [TEK5030] at the University of Oslo.

Please see the [lab guide][guide] for more information.

---

## Prerequisites

As in [lab-stereo], you will need to install the [tek5030 camera-library] for the lab. Head over to the repository and follow the installation instructions.   
Note: The camera-library is preinstalled on the lab computers and in the [`tek5030/devcontainer`] Docker image.

For this lab, we can unfortunately not rely on conan to install all required OpenCV modules (namely the `viz` module for 3D visualization). You have a few other options:

- Solve the [python lab](https://github.com/tek5030/lab-simple-vo-py) (recommended)
- Use the lab computers
- Install OpenCV using [homebrew](https://brew.sh/) (option for mac and linux). (See also [Getting started on MacOS](https://tek5030.github.io/tutorial/macos.html).)
- Try [Docker toolchain][docker-toolchain] in CLion (fairly experimental)
- Try [devcontainer][devcontainer] in VS Code (fairly experimental)
- Rely on virtualbox and our prepared linux image with dependencies preinstalled (see [Canvas: Setting up your computer for the labs](https://uio.instructure.com/courses/44675/discussion_topics/295673))


[repo]:  https://github.com/tek5030/lab-simple-vo
[guide]: https://github.com/tek5030/lab-simple-vo/blob/master/README.md

[TEK5030]: https://www.uio.no/studier/emner/matnat/its/TEK5030/
[conan]: https://tek5030.github.io/tutorial/conan.html
[lab_intro]: https://github.com/tek5030/lab-intro/blob/master/cpp/lab-guide/1-open-project-in-clion.md#6-configure-project
[docker-toolchain]: https://tek5030.github.io/tutorial/docker-toolchain.html
[devcontainer]: https://tek5030.github.io/tutorial/devcontainer.html

[lab-stereo]: https://github.com/tek5030/lab-stereo
[tek5030 camera-library]: https://github.com/tek5030/camera-library
[`tek5030/devcontainer`]: https://hub.docker.com/r/tek5030/devcontainer