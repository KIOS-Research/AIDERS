


<a href="http://www.kios.ucy.ac.cy"><img src="https://www.kios.ucy.ac.cy/wp-content/uploads/2021/07/Logotype-KIOS.svg" width="200" height="100"/><a>


# AIDERS Toolkit


The AIDERS AI toolkit offers various tools for collecting and analyzing emergency response data from UAV sensors. This enables incident commanders to extract knowledge about the field’s operational conditions and assist them in designing evidence-based response strategies. The AI toolkit is delivered in a containerized environment (Docker), along with an android application that is responsible for communicating with the UAV.

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

and   with the help of the following contributors:
* [Rafael Makrigiorgis](https://cy.linkedin.com/in/rafael-makrigiorgis) - Research and Developer Engineer at [KIOS](http://www.kios.ucy.ac.cy/)
* [Andreas Anastasiou](https://cy.linkedin.com/in/rafael-makrigiorgis) - PhD Student at [KIOS](http://www.kios.ucy.ac.cy/)

We would also like to thank the following external resources and libraries, which were used in the development of this software:
* [MapLibre GL JS](https://github.com/maplibre/maplibre-gl-js)
* [Threebox](https://github.com/jscastro76/threebox)
* [OpenDroneMap](https://www.opendronemap.org/)
* [DeckGL](https://deck.gl/)

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

## Demo Video

Click [here]( https://www.kios.ucy.ac.cy/aiders/d6-9-video-clip-on-trials-of-the-ai-toolkit-in-rpas-operation-for-emergency-response/) to view a demonstration video that shows most of AIDERS functionalities

&uparrow; [Back to top](#table-of-contents)
