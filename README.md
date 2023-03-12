<a name="readme-top"></a>

<!-- PROJECT LOGO -->
<br />
<div align="center">


  <h1 align="center"> A-star Algorithm on Turtlebot3</h1>


</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#demo">Demo</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#license">License</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project



This project involved the implementation of the A* algorithm for path planning of a Turtlebot3 robot in a simulated environment using ROS and Gazebo.

Summary of tasks achieved:
* Implemented the A* algorithm for path planning in ROS and Gazebo simulation
* Defined the environment with static obstacles using the half-plane method
* Incorporated the nonholonomic constraint imposed by the Turtlebot3 during path planning
* Navigated the robot to a specific destination using an open loop controller.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Demo

<div align="center">


  <h4 align="center"> Turtlebot3 Simulation (X4 Speed)</h4>


</div>

https://user-images.githubusercontent.com/90359587/224526897-9f21c050-b05d-4b39-8873-e04a1f46ee38.mp4
<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

These are the instructions to get started on the project.
To get a local copy up and running follow these simple steps.

### Prerequisites
* atleast Python 2.0
* Turtlebot3 packages [Installation](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
* OS - Linux (tested)


### Installation

1. Copy the package ```turtlebot3_a_star``` inside ```src``` of catkin workspace . 
2. Run the following commands
   ```sh
   cd ∼ /catkin_ws
   export TURTLEBOT3_MODEL=burger
   catkin build turtlebot3_a_star
   source devel/setup.bash
   ```


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

How to start simulation
1. In the terminal, launch gazebo using the command ('X','Y' and 'Z' should be replaced)
   ```sh
   roslaunch turtlebot3_a_star a_star.launch x_pos:=X y_pos:=Y z_pos:=Z
   ```
   ```sh
   - replace 'X' with values inbetween 0 and 10 . (in metres) 
   - replace 'Y' with values inbetween 0 and 10 . (in metres)       
   - replace 'Z' with values inbetween 0 and 360 . (in degrees)
   ```
2. Open new terminal window and navigate to ```test.py```
   ```sh
   cd catkin_ws/src/turtlebot3_a_star/src 
   ```
   ```sh
   python test.py   
   ```
* Enter data as prompted( start and goal node).
* The ***start node*** should have same values as  ```X``` , ```Y``` and ```Z```, in ***roslaunch*** command. 
* Make sure to enter integer values when asked for, and the value of orientation for start should be in degrees.
* For start and goal positions:

    1. First, input the x - coordinate and press ```Enter``` .Followed by inputing the y-coordinate and press ```Enter``` 
* To terminate. Press ```Ctrl+Z``` on the second terminal window and ```Ctrl+C``` on the first window.

### IMPORTANT NOTE
* If the start ot goal position lies in obstacle space. ***Rerun all the steps under ```Usage``` section***
* Finding path takes less than 45 seconds for the largest distance.
* Test Case :
    1. Start position: (1,1) and orientation = 0 degrees
    2. Goal Position : (9,9)
<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Jeffin Johny K - [![MAIL](https://img.shields.io/badge/Gmail-D14836?style=for-the-badge&logo=gmail&logoColor=white)](mailto:jeffinjk@umd.edu)
	
[![portfolio](https://img.shields.io/badge/my_portfolio-000?style=for-the-badge&logo=ko-fi&logoColor=white)](https://github.com/KACHAPPILLY2021)
[![linkedin](https://img.shields.io/badge/linkedin-0A66C2?style=for-the-badge&logo=linkedin&logoColor=white)](http://www.linkedin.com/in/jeffin-johny-kachappilly-0a8597136)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See [MIT](https://choosealicense.com/licenses/mit/) for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=for-the-badge
[contributors-url]: https://github.com/othneildrew/Best-README-Template/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=for-the-badge
[forks-url]: https://github.com/othneildrew/Best-README-Template/network/members
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=for-the-badge
[stars-url]: https://github.com/othneildrew/Best-README-Template/stargazers
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge
[issues-url]: https://github.com/othneildrew/Best-README-Template/issues
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=for-the-badge
[license-url]: https://github.com/othneildrew/Best-README-Template/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/othneildrew
[product-screenshot]: images/screenshot.png
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com
