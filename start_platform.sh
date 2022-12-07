#!/bin/bash




# We want to avoid using "sudo" with docker. This is required in order to activate Docker BuildKit.
# To do this, we have to add the current $USER to the docker group
# What is BuildKit? https://docs.docker.com/develop/develop-images/build_enhancements/
add_user_to_docker_group()
{
	GROUP=docker
	if grep -q "$GROUP" /etc/group; then #Check if the docker group exists
		 
		 if id -nG "$USER" | grep -qw "$GROUP"; then #Check if current user is actually a member of this group
		 	:  #Do nothing. $USER belongs to $GROUP as it is supposed to
		else
			zenity --warning \
			--text="The current user ($USER) is not a member of the $GROUP group. \nTo add it, follow the instructions here https://docs.docker.com/engine/install/linux-postinstall/ " \
			--width 600 \
			--height 200
			docker stop web
			exit 1
		fi
	else
	    	 zenity --warning \
			--text="Docker has to have the ability to be used without sudo. \nTo do that, click here https://docs.docker.com/engine/install/linux-postinstall/ and follow the instructions"
		docker stop web
	    	 exit 1
    	fi
}



disable_sudo_usage_with_docker()
{
	# Current user has to be added to docker group, in order to avoid using sudo
	add_user_to_docker_group 
	
	#Sometimes, adding the user to the docker group is not enough to avoid sudo
	#For this reason, the following code does the trick
	sudo chmod 666 /var/run/docker.sock
}


# Iterates over the .env.dev file and tries to find the variable user provided, and updates it accordingly with the passed value
write_key_value_to_env_file()
{
	passed_key=$1
	passed_value=$2
	key_found=false
	while IFS='=' read -r key value
	do
	    
	    if [ "$key" = "${passed_key}"  ]; then #Found the key variable variable on the .env file
	       key_found=true

	       if [ -n "${passed_value}" ]; then #If the value of passed variable key is NOT empty
		       if [ "$value" != "${passed_value}" ]; then #If the passed variable has a value different than the one that already exists
		       	sed -i "s|$key=$value|$passed_key=${passed_value}|g" $envFile #Find and replace this variable with the one that was passed here
		       fi
	       fi
	    fi
	done < $envFile

	#If, for some reason, the provided variable was not found at all in the .env file, append it
	if [ "$key_found" = false ] ; then
	    echo "${passed_key}=${passed_value}" >> $envFile
	fi
}


# Sets the udev rules. These rules are responsible for creating a symlink
# for the USB port. This symlink refers to specific USB devices that can be shared with the docker.
# In this way, we can get data from USB ports while being inside docker (e.g from the weather station)
# Reference: https://unix.stackexchange.com/a/66916
set_udev_rules()
{


	####ADDING UDEV RULE FOR WEATHER STATION####
	var_1='SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6015\", SYMLINK+=\"weatherStation\"'

	sudo bash -c "echo $var_1> /etc/udev/rules.d/99-usb-serial.rules"


	####ADDING UDEV RULE FOR LORA STATION####
	var_1='SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"7523\", SYMLINK+=\"loraStation\"'

	sudo bash -c "echo $var_1>> /etc/udev/rules.d/99-usb-serial.rules"


	sudo udevadm control --reload-rules && sudo udevadm trigger

}



#Checks if user provided its network ip
check_for_IP()
{
	if [ -z "${net_ip}" ];then
		echo -e "WARNING: No IP provided. Provide your IP using the following format: ./start_patform.sh --netip '192.168.1.2' "
		exit

  	else
        	write_key_value_to_env_file NET_IP ${net_ip} #Write the provided IP to the env file
	fi

}


#Starts the docker containers with docker-compose up
start_docker_containers_in_background()
{
	echo "Starting Docker Containers..."
	
	sed -i '/runtime: nvidia/d' $dockerComposeFile #If this line exists in docker-compose, remove it no matter what
	existingString='  web:'
	newString='  web:\n    runtime: nvidia'
	sed -i -e "s/$existingString/$newString/g" $dockerComposeFile # Try to insert the "runtime: nvidia" line in the docker-compose file.
	
	
	nvidiaAvailable=1
	nvidiaNotAvailable=0
	
	#Check if nvidia drivers are present
	if (lsmod | grep -q nvidia);then
		write_key_value_to_env_file NVIDIA_AVAILABLE $nvidiaAvailable
		NVIDIA_AVAILABLE=1
	else
		write_key_value_to_env_file NVIDIA_AVAILABLE $nvidiaNotAvailable
		NVIDIA_AVAILABLE=0
	fi

	
	
	modifyFileToWorkWithGPUOrNot $NVIDIA_AVAILABLE

	xhost +local:docker
	docker-compose kill
	docker-compose up -d
	echo "Containers Started! "
}

#Builds and then starts the docker containers
build_dockers()
{
	echo "Building Docker files..."
	docker-compose build
}
#In case the web container is up and running, we can now enter the bash shell of the web container
start_ros_services()
{	

	docker exec -it web bash  -c '${APP_DIR}/start_ros_services.sh; source ${APP_DIR}/django_api/logic/djiswarmcatkinws/devel/setup.bash; cd  ${APP_DIR}/django_api; bash;'
	#docker exec -it web bash  -c '${APP_DIR}/start_ros_services.sh;'
}

# Prints the usage of this script
usage() 
{
	echo 'help:'
        echo '	-b builds the docker'
        echo '	-netip the ip where the ground station server will be started'
        exit 1
}


modifyFileToWorkWithGPUOrNot()
{
	if [ $NVIDIA_AVAILABLE -eq 1 ]; then
		echo "Nvidia available! Will run on GPU!"
		
		# Nvidia is available. We have to run the docker with GPU
		# To do this, we have to add the line "runtime: nvidia" to docker-compose.yml file
		# if the runtime: nvidia does not oalready exist, then append it
		if ! grep -Fxq "    runtime: nvidia" $dockerComposeFile	
		then
			#Find the string 'web:' and replace it with the string 'web:\n runtime:nvidia'
			echo "trying to find web.."
			existingString="  web:"
			newString="  web:\n    runtime: nvidia"
		   	#sed -i -e "s/  web:/  web:\n    runtime: nvidia/g" "$dockerComposeFile"
		   	sed -i 's^$existingString^$newString^g' $dockerComposeFile
		fi
	else
		# We are on a computer that does not have NVIDIA drivers 
		# Check if the runtime nvidia text exists and replace it with nothng
		  echo "Nvidia NOT available. Will run on CPU!"
	  	  #sed -i "s|runtime: nvidia||" $dockerComposeFile
	  	  #sed -i "s^runtime: nvidia^^g" $dockerComposeFile
	  	  sed -i '/runtime: nvidia/d' $dockerComposeFile
	  	  
	fi
}



install_package_if_not_exists()
{
	pkg=$1
	if sudo -dpkg-query -W -f='${Status}' ${pkg}  | grep "ok installed"; 
	then 
		
		:
	else
		echo "Package ${pkg} not installed. Installing now..."
	 	apt-get install ${pkg}
	fi
}
install_required_packages()
{
	install_package_if_not_exists lshw
	
}

open_browser()
{


  if [ -n "$(command -v google-chrome)" ]; then
    google-chrome ${net_ip}:8000
  elif [ -n "$(command -v firefox)" ]; then
    browser_msg_warning "Firefox"
    firefox ${net_ip}:8000
  fi

}


browser_msg_warning()
{
  browser_used=$1
  zenity --warning \
			--text="It seems like you don't have Google Chrome installed on your machine. Chrome was observed to be more faster and was actively tested.\n\nBy clicking OK you are about to open the AIDERS platform using $browser_used. Some visuals might not be displayed correctly.\n\nAlternatively, you can install Google Chrome and run the platform again" \
			--width 600 \
			--height 200
}

initEnvFile()
{
	env_file=$1
  echo "Initializing environment file..."
  if [[ ! -f env_file ]]
  then
    touch "$env_file"
  fi
  ROS_IP=${net_ip}
  ROS_PORT=11311
  ROS_MASTER_URI="http://${ROS_IP}:${ROS_PORT}"
  DEBUG=1
  DJANGO_ALLOWED_HOSTS="* localhost 127.0.0.1"
  SECRET_KEY="django-insecure-7u@(b_01go-msdw=*smjl0(4+02scu=&)m-(*6x%9+wp4tik$^"
  SQL_ENGINE="django.contrib.gis.db.backends.mysql"
  SQL_DATABASE="testdb"
  SQL_USER="test_user"
  SQL_PASSWORD="test_password"
  SQL_PORT="3306"
  DJANGO_PORT="8000"
  DATABASE="mysql"

  write_key_value_to_env_file ROS_IP $ROS_IP
  write_key_value_to_env_file ROS_MASTER_URI $ROS_MASTER_URI
  write_key_value_to_env_file SQL_HOST $HOSTNAME
  write_key_value_to_env_file DISPLAY $DISPLAY
  write_key_value_to_env_file ROS_PORT '11311'

  write_key_value_to_env_file DEBUG $DEBUG
  write_key_value_to_env_file DJANGO_ALLOWED_HOSTS "$DJANGO_ALLOWED_HOSTS"
  write_key_value_to_env_file SECRET_KEY "$SECRET_KEY"
  write_key_value_to_env_file SQL_ENGINE "$SQL_ENGINE"
  write_key_value_to_env_file SQL_DATABASE "$SQL_DATABASE"
  write_key_value_to_env_file SQL_USER "$SQL_USER"
  write_key_value_to_env_file SQL_PASSWORD "$SQL_PASSWORD"
  write_key_value_to_env_file SQL_PORT "$SQL_PORT"
  write_key_value_to_env_file DJANGO_PORT "$DJANGO_PORT"
  write_key_value_to_env_file DATABASE "$DATABASE"

}




envFile=".env"
dockerComposeFile="docker-compose.yml"

	

	   	



# Transform long options to short ones
for arg in "$@"; do
  shift
  case "$arg" in
    '--build')   set -- "$@" '-b'   ;;
    '--netip') set -- "$@" '-n'   ;;
    *)          set -- "$@" "$arg" ;;
  esac
done

net_ip=""






# Iterates over all parameters the user provided to start this bash script
# and decides what to do in each case
while getopts ":b:n:" arg; do
    case "${arg}" in
        b)
          #s=${OPTARG}
          #echo "here"
          build_dockers
          start_docker_containers_in_background
          sleep 5
          start_ros_services
          ;;
        n)
        	net_ip=${OPTARG}
        	
        	;;
		
        *)
            usage
            ;;
    esac
done


# Following code will be executed in case no parameters were provided
	   


initEnvFile $envFile
check_for_IP




set_udev_rules
disable_sudo_usage_with_docker

start_docker_containers_in_background

# Constantly check in background if the site is up and running. Once this happens, automatically open the site on the browser and then exit the while loop
flag=1
(while true; do
   if [[ $(netstat -ano | grep "${net_ip}:8000" | grep LISTEN) && $flag = 1 ]]; then
            flag=0
	    open_browser
	    break
	 fi
         sleep 0.2
done) &

sleep 1

start_ros_services









