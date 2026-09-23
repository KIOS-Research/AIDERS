"""
keycloak_manager.py
Author: Michalis Demetriou
Date: 2025-05-28

This module provides a `KeycloakManager` class to interact with a Keycloak server. 
It includes methods for managing Keycloak entities such as clients, tokens, groups, 
roles, and users.

Key functionalities:
- Authenticate with Keycloak and retrieve an admin token.
- Manage clients, groups, roles, and users in a Keycloak realm.

Dependencies:
- `requests` library for making HTTP requests to the Keycloak server.

Initialization Parameters:
- `url`: Base URL of the Keycloak server.
- `platform_url`: URL of the platform using Keycloak for authentication.
- `realm`: Keycloak realm name.
- `client_uuid`: Client ID for authentication.
- `client_secret`: Client secret for authentication.
- `admin_user`: Keycloak admin username.
- `admin_password`: Keycloak admin password.

Function Input Structures:
1. `create_groups_and_roles(groups_and_roles: List[Dict])`:
   - Accepts a list of dictionaries where each dictionary defines a group and its roles.
   - Example structure:

     groups_and_roles = [
         {
             "group_name": "Admins",
             "roles": ["admin", "manager"]
         },
         {
             "group_name": "Users",
             "roles": ["user"]
         }
     ]

2. `create_users(users: List[Dict])`:
   - Accepts a list of dictionaries where each dictionary defines a user and their attributes.
   - Example structure:

     users = [
         {
             "username": "admin_user",
             "email": "admin@example.com",
             "first_name": "Admin",
             "last_name": "User",
             "password": "securepassword",
             "group": "Admins",
             "is_password_temporary": False,
         },
         {
             "username": "regular_user",
             "email": "user@example.com",
             "first_name": "Regular",
             "last_name": "User",
             "password": "securepassword",
             "group": "Users",
             "is_password_temporary": True,
         }
     ]

Usage:
Import the `KeycloakManager` class and initialize it with the required parameters 
to perform administrative tasks on the Keycloak server, such as creating clients,
groups, roles, and users.

km = KeycloakManager(KEYCLOAK_URL, PLATFORM_URL, "master", KEYCLOAK_CLIENT_ID, KEYCLOAK_CLIENT_SECRET, KEYCLOAK_ADMIN, KEYCLOAK_ADMIN_PASSWORD)
km.create_or_update_client()
km.create_groups_and_roles(groups)
km.create_users(users)
"""

import sys
import requests

class KeycloakManager:

    def __init__(self, keycloak_url, platform_url, realm, client_name, client_secret, admin_user, admin_password):
        self.URL = keycloak_url
        self.PLATFORM_URL = platform_url
        self.REALM = realm
        self.CLIENT_NAME = client_name
        self.CLIENT_SECRET = client_secret
        self.KEYCLOAK_ADMIN = admin_user
        self.KEYCLOAK_ADMIN_PASSWORD = admin_password
        self.REDIRECT_URIS = [f"{platform_url}/accounts/oidc/keycloak/login/callback/"]
        
        self.ICON_SUCCESS = "\u2705"  # Check mark emoji for success messages
        self.ICON_WARNING = "\U0001F7E8"  # Warning emoji for warning messages
        self.ICON_ERROR = "\u274C"  # Cross mark emoji for error messages
        self.ICON_INFO = "\U0001F7E6" # Information emoji for info messages
        
        self.TOKEN = self._get_admin_token()


    # Get an admin token
    def _get_admin_token(self):
        try:
            url = f"{self.URL}/realms/{self.REALM}/protocol/openid-connect/token"
            data = {
                "client_id": "admin-cli",
                "username": self.KEYCLOAK_ADMIN,
                "password": self.KEYCLOAK_ADMIN_PASSWORD,
                "grant_type": "password",
            }
            response = requests.post(url, data=data)
            response.raise_for_status()
            return response.json()["access_token"]
        except Exception as e:
            print(f"{self.ICON_ERROR} Failed to get admin token: {e}")
            sys.exit(1)  # Exit the program with an error code



    ########################
    ### ACTION FUNCTIONS ###
    ########################


    # Create a client
    def _create_client(self):
        print(f"{self.ICON_WARNING} Creating client '{self.CLIENT_NAME}'...")
        try:
            url = f"{self.URL}/admin/realms/{self.REALM}/clients"
            headers = {"Authorization": f"Bearer {self.TOKEN}", "Content-Type": "application/json"}
            data = {
                "clientId": self.CLIENT_NAME,
                "secret": self.CLIENT_SECRET,
                "enabled": True,
                "redirectUris": self.REDIRECT_URIS,
            }
            response = requests.post(url, json=data, headers=headers)
            response.raise_for_status()
            print(f"{self.ICON_SUCCESS} Client '{self.CLIENT_NAME}' created.")
        except Exception as e:
            print(f"{self.ICON_ERROR} Failed to create client '{self.CLIENT_NAME}': {e}")
            sys.exit(1)


    # Update client redirect URIs
    def _update_client_redirect_uris(self):
        print(f"{self.ICON_WARNING} Updating redirect URIs for client '{self.CLIENT_NAME}'...")
        try:
            client_uuid = self._get_client_uuid()
            url = f"{self.URL}/admin/realms/{self.REALM}/clients/{client_uuid}"
            headers = {"Authorization": f"Bearer {self.TOKEN}", "Content-Type": "application/json"}
            data = {"redirectUris": self.REDIRECT_URIS}
            response = requests.put(url, json=data, headers=headers)
            response.raise_for_status()
            print(f"{self.ICON_SUCCESS} Redirect URIs for client '{self.CLIENT_NAME}' updated to: {self.REDIRECT_URIS}")
        except Exception as e:
            print(f"{self.ICON_ERROR} Failed to update redirect URIs for client '{self.CLIENT_NAME}': {e}")
            sys.exit(1)
    

    # Create a group
    def _create_group(self, group_name):
        print(f"{self.ICON_WARNING} Creating group '{group_name}'...")
        try:
            url = f"{self.URL}/admin/realms/{self.REALM}/groups"
            headers = {"Authorization": f"Bearer {self.TOKEN}", "Content-Type": "application/json"}
            data = {"name": group_name}
            response = requests.post(url, json=data, headers=headers)
            response.raise_for_status()
            print(f"{self.ICON_SUCCESS} Group '{group_name}' created.")
        except Exception as e:
            print(f"{self.ICON_ERROR} Failed to create group '{group_name}': {e}")
            sys.exit(1)


    # Create a client role
    def _create_client_role(self, role_name):
        print(f"{self.ICON_WARNING} Creating client role '{role_name}' for client '{self.CLIENT_NAME}'...")
        try:
            client_uuid = self._get_client_uuid()
            url = f"{self.URL}/admin/realms/{self.REALM}/clients/{client_uuid}/roles"
            headers = {"Authorization": f"Bearer {self.TOKEN}", "Content-Type": "application/json"}
            data = {"name": role_name}
            response = requests.post(url, json=data, headers=headers)
            response.raise_for_status()
            print(f"{self.ICON_SUCCESS} Client role '{role_name}' created for client '{self.CLIENT_NAME}'.")
        except Exception as e:
            print(f"{self.ICON_ERROR} Failed to create client role '{role_name}' for client '{self.CLIENT_NAME}': {e}")
            sys.exit(1)


    # Assign a client role to a group
    def _assign_client_role_to_group(self, group_name, role_name):
        print(f"{self.ICON_WARNING} Assigning client role '{role_name}' to group '{group_name}' for client '{self.CLIENT_NAME}'...")
        try:
            headers = {"Authorization": f"Bearer {self.TOKEN}"}
            client_uuid = self._get_client_uuid()
            group_id = self._get_group_id(group_name)
            role = self._get_client_role(role_name)
            url = f"{self.URL}/admin/realms/{self.REALM}/groups/{group_id}/role-mappings/clients/{client_uuid}"
            response = requests.post(url, json=[role], headers=headers)
            response.raise_for_status()
            print(f"{self.ICON_SUCCESS} Role '{role_name}' assigned to group '{group_name}'.")
        except Exception as e:
            print(f"{self.ICON_ERROR} Failed to assign role '{role_name}' to group '{group_name}': {e}")
            sys.exit(1)


    # Create a user and assign them to a group
    def _create_user(self, username, password, email, first_name, last_name, is_password_temporary=False):
        print(f"{self.ICON_WARNING} Creating user '{username}' with email {email}...")
        try:
            url = f"{self.URL}/admin/realms/{self.REALM}/users"
            headers = {"Authorization": f"Bearer {self.TOKEN}", "Content-Type": "application/json"}
            data = {
                "username": username,
                "enabled": True,
                "email": email,
                "firstName": first_name,
                "lastName": last_name,
                "credentials": [{"type": "password", "value": password, "temporary": is_password_temporary}],
            }
            response = requests.post(url, json=data, headers=headers)
            response.raise_for_status()
            print(f"{self.ICON_SUCCESS} User '{username}' created.")

        except Exception as e:
            print(f"{self.ICON_ERROR} Failed to create user '{username}' with email {email}: {e}")
            sys.exit(1)


    # Assign user to group
    def _assign_user_to_group(self, username, group_name):
        print(f"{self.ICON_WARNING} Assigning user '{username}' to group '{group_name}'...")
        try:
            headers = {"Authorization": f"Bearer {self.TOKEN}", "Content-Type": "application/json"}
            user_id = self._get_user_id(username)
            group_id = self._get_group_id(group_name)
            url = f"{self.URL}/admin/realms/{self.REALM}/users/{user_id}/groups/{group_id}"
            response = requests.put(url, headers=headers)
            response.raise_for_status()
            print(f"{self.ICON_SUCCESS} User '{username}' added to group '{group_name}'.")
        except Exception as e:
            print(f"{self.ICON_ERROR} Failed adding user '{username}' to group '{group_name}': {e}")
            sys.exit(1)


    ########################
    ### HELPER FUNCTIONS ###
    ########################


    # Check if a client exists
    def _client_exists(self,):
        print(f"{self.ICON_INFO} Checking if client '{self.CLIENT_NAME}' exists...")
        url = f"{self.URL}/admin/realms/{self.REALM}/clients"
        headers = {"Authorization": f"Bearer {self.TOKEN}"}
        response = requests.get(url, headers=headers)
        response.raise_for_status()
        clients = response.json()
        client_exists = any(client["clientId"] == self.CLIENT_NAME for client in clients)
        if client_exists:
            print(f"{self.ICON_INFO} Client '{self.CLIENT_NAME}' already exists.")
        else:
            print(f"{self.ICON_INFO} Client '{self.CLIENT_NAME}' does not exist.")
        return client_exists


    # Check if a group exists
    def _group_exists(self, group_name):
        print(f"{self.ICON_INFO} Checking if group '{group_name}' exists...")
        url = f"{self.URL}/admin/realms/{self.REALM}/groups"
        headers = {"Authorization": f"Bearer {self.TOKEN}"}
        response = requests.get(url, headers=headers, params={"search": group_name})
        response.raise_for_status()
        groups = response.json()
        group_exists = any(group["name"] == group_name for group in groups)
        if group_exists:
            print(f"{self.ICON_INFO} Group '{group_name}' already exists.")
        else:
            print(f"{self.ICON_INFO} Group '{group_name}' does not exist.")
        return group_exists
    

    # Check if user exists
    def _user_exists(self, username, email):
        print(f"{self.ICON_INFO} Checking if user '{username}' with email {email} exists...")
        url = f"{self.URL}/admin/realms/{self.REALM}/users"
        headers = {"Authorization": f"Bearer {self.TOKEN}"}

        # Fetch users by username
        params_username = {"username": username}
        response_username = requests.get(url, headers=headers, params=params_username)
        response_username.raise_for_status()
        users_by_username = response_username.json()
        # Filter users to match the username exactly
        users_by_username = [user for user in users_by_username if user.get("username") == username]

        # Fetch users by email
        params_email = {"email": email}
        response_email = requests.get(url, headers=headers, params=params_email)
        response_email.raise_for_status()
        users_by_email = response_email.json()
        # Filter users to match the email exactly
        users_by_email = [user for user in users_by_email if user.get("email") == email]

        if len(users_by_username) == 0 and len(users_by_email) == 0:
            print(f"{self.ICON_INFO} User '{username}' with email {email} does not exist.")
            return False
        else:
            print(f"{self.ICON_INFO} User '{username}' with email {email} already exists.")
            return True
    

    # Check if a client role exists
    def _client_role_exists(self, role_name):
        print(f"{self.ICON_INFO} Checking if client role '{role_name}' exists for client '{self.CLIENT_NAME}'...")
        client_uuid = self._get_client_uuid()
        url = f"{self.URL}/admin/realms/{self.REALM}/clients/{client_uuid}/roles"
        headers = {"Authorization": f"Bearer {self.TOKEN}"}
        response = requests.get(url, headers=headers)
        response.raise_for_status()
        roles = response.json()
        role_exists = any(role["name"] == role_name for role in roles)
        if role_exists:
            print(f"{self.ICON_INFO} Client role '{role_name}' already exists for client '{self.CLIENT_NAME}'.")
        else:
            print(f"{self.ICON_INFO} Client role '{role_name}' does not exist for client '{self.CLIENT_NAME}'.")
        return role_exists


    # Get client ID
    def _get_client_uuid(self):
        clients_url = f"{self.URL}/admin/realms/{self.REALM}/clients"
        headers = {"Authorization": f"Bearer {self.TOKEN}"}
        response = requests.get(clients_url, headers=headers, params={"clientId": self.CLIENT_NAME})
        response.raise_for_status()
        clients = response.json()
        if not clients:
            raise ValueError(f"Client '{self.CLIENT_NAME}' not found.")
        return clients[0]["id"]
    

    # Get group ID
    def _get_group_id(self, group_name):
        headers = {"Authorization": f"Bearer {self.TOKEN}"}
        url = f"{self.URL}/admin/realms/{self.REALM}/groups"
        response = requests.get(url, headers=headers, params={"search": group_name})
        response.raise_for_status()
        groups = response.json()
        if not groups:
            raise ValueError(f"Group '{group_name}' not found.")
        return groups[0]["id"]
    

    # Get client role
    def _get_client_role(self, role_name):
        headers = {"Authorization": f"Bearer {self.TOKEN}"}
        client_uuid = self._get_client_uuid()
        url = f"{self.URL}/admin/realms/{self.REALM}/clients/{client_uuid}/roles/{role_name}"
        response = requests.get(url, headers=headers)
        response.raise_for_status()
        return response.json()            


    # Get user ID
    def _get_user_id(self, username):
        url = f"{self.URL}/admin/realms/{self.REALM}/users"
        headers = {"Authorization": f"Bearer {self.TOKEN}", "Content-Type": "application/json"}
        # Get user ID
        response = requests.get(url, headers=headers, params={"username": username})
        response.raise_for_status()
        users = response.json()
        if not users:
            raise ValueError(f"User '{username}' not found.")
        return users[0]["id"]


    ########################
    ### PUBLIC FUNCTIONS ###
    ########################


    def create_or_update_client(self):
        print(f"\n--------------------")
        print(f"\u23F3 Client Operations")
        print(f"--------------------")
        if not self._client_exists():
            self._create_client()
        else:
            self._update_client_redirect_uris()


    def create_groups_and_roles(self, groups_and_roles):
        print(f"\n------------------------------")
        print(f"\u23F3 Groups and Roles Operations")
        print(f"------------------------------")
        for group in groups_and_roles:
            if not self._group_exists(group["name"]):
                self._create_group(group["name"])
            for role in group["roles"]:
                if not self._client_role_exists(role):
                    self._create_client_role(role)
                self._assign_client_role_to_group(group["name"], role)


    def create_users(self, users):
        print(f"\n-------------------")
        print(f"\u23F3 Users Operations")
        print(f"-------------------")
        for user in users:
            if not self._user_exists(user["username"], user["email"]):
                self._create_user(
                    username=user["username"],
                    password=user["password"],
                    email=user["email"],
                    first_name=user.get("first_name", ""),
                    last_name=user.get("last_name", ""),
                    is_password_temporary=user.get("is_password_temporary", False)
                )
            else:
                print(f"{self.ICON_ERROR} User '{user['username']}' with email {user['email']} already exists. Skipping creation.")

            if self._group_exists(user["group"]):
                self._assign_user_to_group(
                    username=user["username"],
                    group_name=user["group"]
                )
            else:
                print(f"{self.ICON_ERROR} Group '{user['group']}' does not exist. Skipping assigning to '{user['username']}'.")

