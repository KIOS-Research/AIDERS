# <img alt="aiders" src="https://user-images.githubusercontent.com/68916468/207874796-fc816922-397b-48c9-8b0b-b341a8758c24.png" width="30%">
![Screenshot from 2022-12-15 13-09-46](https://user-images.githubusercontent.com/68916468/207859204-f047bdd1-357f-4e78-bc88-69da87479033.png)



# AIDERS Toolkit
The AIDERS toolkit is a web-based platform that offers various tools for collecting and analyzing emergency response data from UAV sensors.  The incident commanders can also command, control and view the drones in real time in a 3D map environment. This enables incident commanders to extract knowledge about the field’s operational conditions and assist them in designing evidence-based response strategies. 

There are also several other tools offered that are non-UAV related, such as visualizing static information (e.g a country's infrastructure), create georeferenced 3D objects, or simulate the propagation of a fire.

The toolkit is delivered in a containerized environment (Docker), along with an android application that is responsible for communicating with the UAV. 

All the data are collected and stored locally on the machine the platform is installed.

## Table of Contents

- [AIDERS Toolkit](#aiders-toolkit)
- [How to cite](#how-to-cite)
- [Requirements](#requirements)
- [How to install](#how-to-install)
- [License](#license)
- [Contributors](#contributors)
- [Contributing](#contributing)
- [Data Flow](#data-flow)
- [Disclaimer](#disclaimer)
- [Cool Features](#cool-features)
- [Demo Video](#demo-video)
  
## How to cite 

To be completed.

&uparrow; [Back to top](#table-of-contents)

## Requirements
Since AIDERS runs in a containerized Linux environment, you'd need to have Docker installed and the host machine has to be a Linux distribution:
* [Docker](https://docs.docker.com/engine/install/ubuntu/) (Tested version: 20.10.11)
* Linux OS  (Tested mostly on Linux Ubuntu 20.04, but any Linux distribution should work)
* [Docker Compose](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-compose-on-ubuntu-20-04) (Tested version: 1.29.2)

&uparrow; [Back to top](#table-of-contents)

## How to install

Instructions for installing and running the Docker image for AIDERS, can be found in [AIDERS official site](https://www.kios.ucy.ac.cy/aiders/aiders-ai-toolkit/).

To develop and build your own version, go to `docker-compose.yml` file, and uncomment the volumes that specify the `#development` at the end.

&uparrow; [Back to top](#table-of-contents)




## License

* AIDERS project is EUPL licensed, as found in the LICENSE file.

&uparrow; [Back to top](#table-of-contents)

## Contributors

AIDERS was developed by:
* [Panayiotis Kolios](https://www.kios.ucy.ac.cy/pkolios/) - Research Assistant Professor at [KIOS](http://www.kios.ucy.ac.cy/) and advisor of this project
*  [Giorgos Sofroniou](https://cy.linkedin.com/in/george-sofroniou-b78b001a9) - Research and Developer Engineer at [KIOS](http://www.kios.ucy.ac.cy/)
* [Alexis Piperides](https://cy.linkedin.com/in/alexis-piperides-aba303222) - Research and Developer Engineer at [KIOS](http://www.kios.ucy.ac.cy/)
* [Amal Abed]() - Research and Developer Engineer at [KIOS](http://www.kios.ucy.ac.cy/)

and   with the help of the following contributors:
* [Rafael Makrigiorgis](https://cy.linkedin.com/in/rafael-makrigiorgis) - Research and Developer Engineer at [KIOS](http://www.kios.ucy.ac.cy/)
* [Andreas Anastasiou](https://cy.linkedin.com/in/andreas-anastasiou) - PhD Student at [KIOS](http://www.kios.ucy.ac.cy/)

We would also like to thank the following external resources and libraries, which were used in the development of this software:
* [MapLibre GL JS](https://github.com/maplibre/maplibre-gl-js)
* [Threebox](https://github.com/jscastro76/threebox)
* [OpenDroneMap](https://www.opendronemap.org/)
* [DeckGL](https://deck.gl/)
* [OpenStreet Map Tile Server](https://github.com/Overv/openstreetmap-tile-server)

&uparrow; [Back to top](#table-of-contents)
## Contributing
Everyone is welcome to participate in this project. Whether you are helping others to resolve issues, reporting a new issue that hasn't yet been discovered, suggesting a new feature that would benefit your workflow, or writing code (or tests, or scripts, or ...), we value your time and effort.

The path for contribution starts with the [Issues](https://github.com/KIOS-Research/AIDERS/issues). You can create a new issue in case you discover a new bug or you have a useful suggestion that you’d like to be implemented. If you want to contribute code, fork this repo to your own account. Make your commits on your dev branch (or one based on dev). Once you are finished, you can open a Pull Request to test the code and discuss merging your changes back into the community repository.

&uparrow; [Back to top](#table-of-contents)

## Data Flow
Following diagram shows how the data travel in the local network
<a href="http://www.kios.ucy.ac.cy"><img src="https://www.kios.ucy.ac.cy/aiders/wp-content/uploads/2022/12/data_flow.png" width="auto" height="auto"/><a>

&uparrow; [Back to top](#table-of-contents)

## Disclaimer
This is an open source software, and is provided to you "as is" without any warranties or guarantees. We make no representations or warranties of any kind, express or implied, about the completeness, accuracy, reliability, suitability or availability with respect to the software. Any reliance you place on the information provided by this software is strictly at your own risk.

&uparrow; [Back to top](#table-of-contents)

## Cool Features
* ### Create 3D objects using only 2D drone images as input
  ![Screenshot from 2022-12-15 12-29-51](https://user-images.githubusercontent.com/68916468/207857579-fd83faf1-fd9c-4863-99d3-dba4b5157f7f.png)
  ![Screenshot from 2022-12-15 12-25-56](https://user-images.githubusercontent.com/68916468/207857478-cc115bea-2eae-4471-b01d-936327f221ae.png)

* ### Send a swarm of drones to follow a path autonomously with a click of a button
  ![Screenshot from 2022-12-15 12-52-29](https://user-images.githubusercontent.com/68916468/207861893-8c03989a-159f-4fb2-9b75-812350aa6afc.png)
  ![Screenshot from 2022-12-15 12-52-41](https://user-images.githubusercontent.com/68916468/207861900-2a40708b-e7ef-465a-ba19-75b4a0080170.png)

* ### Detect vehicles or people and georeference them on the map in real time
  ![Screenshot from 2022-12-15 13-01-03](https://user-images.githubusercontent.com/68916468/207870416-45898ac8-5e0a-41b2-8c08-0eb1d6a2a416.png)
  ![Screenshot from 2022-12-15 12-53-51](https://user-images.githubusercontent.com/68916468/207870437-95c3b3ab-836a-4f6a-8556-e9228de665b5.png)

* ### Build an area of interest in real-time using a UAV
  ![Screenshot from 2022-12-15 13-02-18](https://user-images.githubusercontent.com/68916468/207870079-641e3a82-a820-4a37-9285-9052a95227bd.png)

* ### Download and serve your own map tiles for a country or region to operate in offline mode
  ![Screenshot from 2022-12-15 12-51-44](https://user-images.githubusercontent.com/68916468/207870786-b04681f7-d2da-454b-99d5-bf30c99a9f16.png)

* ### Create users and operations with permissions
  ![Screenshot from 2022-12-15 12-35-39](https://user-images.githubusercontent.com/68916468/207871391-d84e7f50-b5bc-4846-98d5-b4a369c79579.png)

* ### Simulate a fire propagation
  ![Screenshot from 2022-12-15 12-32-21](https://user-images.githubusercontent.com/68916468/207876457-e7a1d6bf-581a-49a2-bef4-887544bf902e.png)
  ![Screenshot from 2022-12-15 12-32-30](https://user-images.githubusercontent.com/68916468/207876474-5e57ad0b-8f0f-4e63-ab3f-07bd1b0f9e33.png)

* ### Enabling several features together to gain full situational awareness in real time
  ![Screenshot from 2022-12-15 13-09-46](https://user-images.githubusercontent.com/68916468/207871526-5177ff82-b6bf-4c2e-a90b-4a1d440e0319.png)

* ### Measure length or area of selected points
  ![Screenshot from 2022-12-15 12-50-50](https://user-images.githubusercontent.com/68916468/207869530-806a20ea-3a67-40c0-95a1-3f34ef608a39.png)

## Demo Video

Click [here]( https://www.kios.ucy.ac.cy/aiders/d6-9-video-clip-on-trials-of-the-ai-toolkit-in-rpas-operation-for-emergency-response/) to view a demonstration video that shows most of AIDERS functionalities

&uparrow; [Back to top](#table-of-contents)
