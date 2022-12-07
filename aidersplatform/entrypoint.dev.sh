#!/bin/bash


# Future work: 
# 1) Check every tool this script uses and inform user to install it if it doesn't exist. See Pycharm.sh how they do it
# 2) Don't open the url with firefox, but with the linux's default browser


is_first_execution()
{
  CONTAINER_ALREADY_STARTED="CONTAINER_ALREADY_STARTED_PLACEHOLDER"
  if [ ! -e $CONTAINER_ALREADY_STARTED ]; then
    touch $CONTAINER_ALREADY_STARTED
    echo "-- First container startup --"
    # 0 = true
    return 0 
  else
    # 1 = false
    return 1
  fi
}

# Make sure at least one super user exists in the database. If not inform the user about it and let them know that one has to be 
# created in order to use the platform. The reason is because if no superuser exists then no other user accounts can be created
ensure_superuser()
{
	superUserExists=$(python manage.py ensure_adminuser  --superUserExists)

	
	if [ "$superUserExists" = True ] ; then
		echo "At least one Super User exists"
	else
		zenity --question \
		--title="No Super User found" \
		--text "It seems like there is no super user created for the AIDERS platform. \nIf this the first time the platform is executed, a super user needs to be created. \nThis user will have all permissions, including creating or removing other users and modifying their permissions. Click OK to proceed and create the super user" \
		--ok-label="Cancel" \
		--cancel-label="OK" \
		--width 600 \
		--height 200
		
	canProceed=$?
			
		if [ $canProceed == 1 ]; then
			ENTRY=`zenity --forms --title="Superuser Details" \
			--text "Create Super User" \
			--separator="," \
			--add-entry="Username" \
			--add-entry="First Name" \
			--add-entry="Last Name" \
			--add-entry="Email" \
			--add-password="Password" \
			--width 200 \
		-	--height 300`

			case $? in
			    0)
				USERNAME=$(echo $ENTRY | cut -d',' -f1)
				FIRSTNAME=$(echo $ENTRY | cut -d',' -f2)
				LASTNAME=$(echo $ENTRY | cut -d',' -f3)
				EMAIL=$(echo $ENTRY | cut -d',' -f4)
				PASSWORD=$(echo $ENTRY | cut -d',' -f5)
				;;
			    1)
				echo "No Data recorded."
				kill -SIGKILL $$
				exit
				;;
			    -1)
				echo "An unexpected error has occurred."
				kill -SIGKILL $$
				exit
				;;
			esac
			echo "ABOUT TO CREATE AN ADMIN USER"
			python manage.py ensure_adminuser   --username=${USERNAME} --email=${EMAIL} --password=${PASSWORD} --firstname=${FIRSTNAME} \
			--lastname=${LASTNAME} --createSuperuser
			
		else
			kill -SIGKILL $$
			exit
		fi
	fi
}

(
# =================================================================
echo "# Starting database..." ;
#Wait until mysql is started
if [ "$DATABASE" = "mysql" ]
then
    echo "Waiting for mysql..."

    while ! nc -z $SQL_HOST $SQL_PORT; do
      sleep 0.1
    done

    echo "mysql started"
fi
# Command for first task goes on this line.

# =================================================================
echo "25"
echo "# Setting up the environment..." ;

#Activate the ros noetic environment
source /opt/ros/noetic/setup.bash

#Activate our custom ROS environment
cd $APP_DIR/django_api/logic/djiswarmcatkinws
chmod +x $APP_DIR/django_api/logic/djiswarmcatkinws/devel/setup.bash
source $APP_DIR/django_api/logic/djiswarmcatkinws/devel/setup.bash

# =================================================================
echo "50"
echo "# Initializing database..." ;
cd $APP_DIR/django_api
python3 manage.py makemigrations aiders

# =================================================================
echo "62"
echo "# Creating database tables..." ;
python3 manage.py migrate

# =================================================================
echo "75"
echo "# Checking if admin account exists..." ;
ensure_superuser


# =================================================================
echo "# Platform started!" ;
echo "100"


) |
zenity --progress \
  --title="Progress Status" \
  --text="First Task." \
  --percentage=0 \
  --auto-close

if [ "$?" = -1 ] ; then
        zenity --error \
          --text="Update canceled."
        kill -SIGKILL $$
				exit
fi

cd $APP_DIR/django_api/logic/djiswarmcatkinws
chmod +x $APP_DIR/django_api/logic/djiswarmcatkinws/devel/setup.bash
source $APP_DIR/django_api/logic/djiswarmcatkinws/devel/setup.bash
cd $APP_DIR/django_api
exec "$@"








