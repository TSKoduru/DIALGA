<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/TSKoduru/DIALGA">
    <img src="assets/logo.png" alt="Logo" width="200" height="200">
  </a>

<h3 align="center">DIALGA</h3>

  <p align="center">
    An open-source binary watch to make at home! (WIP)
    <br />
    <a href="https://github.com/TSKoduru/DIALGA"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/TSKoduru/DIALGA">View Demo</a>
    ·
    <a href="https://github.com/TSKoduru/DIALGA/issues">Report Bug</a>
    ·
    <a href="https://github.com/TSKoduru/DIALGA/issues">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
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
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

NOTE: This project is still under active development. The software needs to be tested, and the case needs to be designed. I'll update this page with more information as I make progress. Please don't jump into manufacturing this watch just yet. I'll let you know when it's ready. Feel free to look around, though!


![3D-View][3D-View-Top]


<p align="right">(<a href="#readme-top">back to top</a>)</p>

I saw an article at the start of summer about a cool little watch that somebody had made for themselves. It told time in binary format, and I thought it looked pretty cool. So, I decided to make my own version of it, so I could learn PCB design, soldering, and embedded programming. I also wanted to make it open-source, so that others could make their own versions of it, and learn from my mistakes. Speaking of which, I made a lot of mistakes. There's probably a few things to improve, so if you have any suggestions, feel free to open an issue or a pull request. Thanks for checking out my project!

### Built With

* [![Altium Designer](https://img.shields.io/badge/Altiuim-AD20-blue)](https://www.altium.com/)
* [![Arduino](https://img.shields.io/badge/Arduino-IDE-blue)](https://www.arduino.cc/)



<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

If you're looking to make your own version of this project, you'll need to do the following:

* Get the GERBER and drill files from the COMPRESSED folder, and send them to a PCB manufacturer. I used JLCPCB, and they worked great. You can find the link to their website [here](https://jlcpcb.com/quote). If you want to modify the PCB, you're going to want to download the .PCB files and open them in your editor of choice.

* Order the parts from the BOM file. You can find the BOM file in the BOM folder. I ordered my parts from Digikey, but you can order them from wherever you want. Pro tip: The RTC module is super expensive (Cost me ~11 dollars), so if you want to save a few dollars, try to find someone selling it for cheap.

* Solder the parts onto the PCB. You can use a soldering iron for this, but it'd go a lot faster if you use a hot plate or a reflow oven. I used a hot plate, and it worked great. Just make sure to use a lot of flux, and you should be fine.

* Program the Arduino. You can find the code in the Arduino folder. Just open the .ino file in the Arduino IDE, and upload it to your Arduino. The process to do this is a little complicated, so I'd recommend looking up a tutorial. It involves flashing a bootloader to the chip from another atmega chip, and then using an adapater to upload the program.

* If you want to make a custom case, the 3D file for the PCB is located in the step folder.


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

The controls for this watch are pretty simple:


* The watch is meant to be worn on the left hand. All instructions will be given assuming that you are wearing it on your left hand.

* Firstly, power the watch with 3.3V lithium coin cell batteries. The watch is configured to go into deep sleep when not in use, so it should last a while.

* If the battery is low, a special LED near the bottom right of the watch will light up. If this LED is on, you should change out the battery as soon as possible.

* Both switches are located on the right side of the watch. The top switch is the ON/OFF switch, and the bottom switch is the MODE switch.

* To use the watch, simply press the ON/OFF switch. The watch will turn on, and the time will be displayed. If you want to see the date or temperature, press the mode switch. The corresponding 'state' LEDs, located at the top right of the board, will light up to show you what state you are in.

* If you need to set the time (Which you'll need to do after changing the batteries), press the mode switch 5 times. The watch will enter the time setting mode, and the time will start flashing. Press the mode switch to cycle through the hours, minutes, and seconds. Press the ON/OFF switch to increment the selected value. Press the ON/OFF switch to confirm the value, and move on to the next one.

* Again, information is always displayed in binary, with the left column being the most significant bit, and the right column being the least significant bit. The time is displayed in 24-hour format, and the date is displayed in the format DD/MM/YYYY.


<!-- ROADMAP -->
## Roadmap

- [ ] Order and manufacture the prototype.
- [ ] Test the prototype software and hardware (Specifically the watch case).
- [ ] Begin manufacturing the final product.

See the [open issues](https://github.com/TSKoduru/DIALGA/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Teja koduru - [@TJKoduru](https://twitter.com/TJKoduru) - tkoduru@gmail.com

Project Link: [https://github.com/TSKoduru/DIALGA](https://github.com/TSKoduru/DIALGA)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [Sverd Industries](https://www.instructables.com/Binary-Wrist-Watch/)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TSKoduru/DIALGA.svg?style=for-the-badge
[contributors-url]: https://github.com/TSKoduru/DIALGA/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TSKoduru/DIALGA.svg?style=for-the-badge
[forks-url]: https://github.com/TSKoduru/DIALGA/network/members
[stars-shield]: https://img.shields.io/github/stars/TSKoduru/DIALGA.svg?style=for-the-badge
[stars-url]: https://github.com/TSKoduru/DIALGA/stargazers
[issues-shield]: https://img.shields.io/github/issues/TSKoduru/DIALGA.svg?style=for-the-badge
[issues-url]: https://github.com/TSKoduru/DIALGA/issues
[license-shield]: https://img.shields.io/github/license/TSKoduru/DIALGA.svg?style=for-the-badge
[license-url]: https://github.com/TSKoduru/DIALGA/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/TSKoduru
[3D-View-Top]: assets/3D-View-Top.png
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
