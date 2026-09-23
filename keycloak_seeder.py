"""
keycloak_seeder.py
Author: Michalis Demetriou
Date: 2025-05-28

This script is used to seed Keycloak with predefined groups, roles, and users 
for the platform. It reads configuration values from a `.env` file and uses 
the KeycloakManager class to interact with the Keycloak server.

Dependencies:
- `python-dotenv` for loading environment variables.
- `keycloak_manager` custom module for managing Keycloak interactions.

Ensure the following environment variables are set in the `.env` file:
- NET_IP, NGINX_PORT, KEYCLOAK_PORT, KEYCLOAK_DJANGO_CLIENT_ID, 
  KEYCLOAK_DJANGO_CLIENT_SECRET, KEYCLOAK_ADMIN, KEYCLOAK_ADMIN_PASSWORD, 
  ADMIN_USER, ADMIN_PASSWORD.

Usage:
Adjust the contents of the "groups" and "users" arrays and run this script
to seed them to Keycloak.
"""

from dotenv import load_dotenv
import os
import sys
from keycloak_manager import KeycloakManager


# Load environment variables from the .env file
current_folder = os.path.dirname(os.path.abspath(__file__))
dotenv_path = os.path.join(current_folder, '.env')
load_dotenv(dotenv_path)

PLATFORM_URL = f"http://{os.getenv('NET_IP')}:{os.getenv('NGINX_PORT')}"
KEYCLOAK_URL = f"http://{os.getenv('KEYCLOAK_IP')}:{os.getenv('KEYCLOAK_PORT')}"
KEYCLOAK_CLIENT_ID = os.getenv("KEYCLOAK_DJANGO_CLIENT_ID")
KEYCLOAK_CLIENT_SECRET = os.getenv("KEYCLOAK_DJANGO_CLIENT_SECRET")
KEYCLOAK_ADMIN = os.getenv("KEYCLOAK_ADMIN")
KEYCLOAK_ADMIN_PASSWORD = os.getenv("KEYCLOAK_ADMIN_PASSWORD")
PLATFORM_ADMIN_USER = os.getenv("ADMIN_USER")
PLATFORM_ADMIN_PASSWORD = os.getenv("ADMIN_PASSWORD")



# Define the groups and roles to be created
groups = [
    {
        "name": "aiders-superusers",
        "roles": [
            "superuser",
        ],
    },      
    {
        "name": "aiders-admins",
        "roles": [
            "view_operations",
            "manage_operations",
            "manage_users",
        ],
    },    
    {
        "name": "aiders-users",
        "roles": [
            "view_operations",
            "manage_operations",
        ],
    },
    {
        "name": "aiders-viewers",
        "roles": [
            "view_operations",
        ],
    },
]

# Define the users to be created
users = [
    {
        "username": PLATFORM_ADMIN_USER,
        "password": PLATFORM_ADMIN_PASSWORD,
        "email": "admin@test.com",
        "first_name": "Admin",
        "last_name": "Tester",        
        "group": "aiders-superusers",
        "is_password_temporary": False,
    },
    {
        "username": "aiders-user1",
        "password": "aiders-user-password",
        "email": "user1@test.com",
        "first_name": "User",
        "last_name": "One",        
        "group": "aiders-users",
        "is_password_temporary": True,
    },
    {
        "username": "aiders-user2",
        "password": "aiders-user-password",
        "email": "user2@test.com",
        "first_name": "User",
        "last_name": "Two",
        "group": "aiders-users",
        "is_password_temporary": True,
    },
    {
        "username": "aiders-viewer",
        "password": "aiders-user-password",
        "email": "viewer@test.com",
        "first_name": "Viewer",
        "last_name": "One",
        "group": "aiders-viewers",
        "is_password_temporary": True,
    }    
]


# Initialize KeycloakManager
km = KeycloakManager(KEYCLOAK_URL, PLATFORM_URL, "master", KEYCLOAK_CLIENT_ID, KEYCLOAK_CLIENT_SECRET, KEYCLOAK_ADMIN, KEYCLOAK_ADMIN_PASSWORD)

if "--client-only" in sys.argv:
    print("Creating Keycloak client only...")
    km.create_or_update_client()
else:
    print("Seeding Keycloak with groups and users...")
    km.create_or_update_client()
    km.create_groups_and_roles(groups)
    km.create_users(users)