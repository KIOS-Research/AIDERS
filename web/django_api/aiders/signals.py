from django.contrib.auth.signals import user_logged_in
from django.dispatch import receiver
from django.contrib.auth import logout
from django.shortcuts import redirect
from .models import User, UserPreferences
from pprint import pprint
import os
import jwt
from django.contrib.auth.models import Permission
from allauth.socialaccount.models import SocialToken

@receiver(user_logged_in)
def post_login_hook(sender, request, user, **kwargs):
    print("+++++++++++++++ post_login_hook +++++++++++++++", flush=True)

    # Retrieve the SocialToken for the logged-in user
    social_token = SocialToken.objects.filter(account__user=user, account__provider='keycloak').first()
    if not social_token:
        print("No SocialToken found for the user.", flush=True)
        logout(request)  # Log out the user if token decoding fails
        return redirect("/accounts/oidc/keycloak/login/")

    # Decode the ID token or access token
    try:
        token = social_token.token
        decoded_token = jwt.decode(token, options={"verify_signature": False})  # Disable signature verification for testing
        print(f"Decoded Token: {decoded_token}", flush=True)

        # Extract roles from the token
        realm_roles = decoded_token.get('realm_access', {}).get('roles', [])
        print(f"Realm Roles: {realm_roles}", flush=True)

        # If client-specific roles are needed
        client_name = os.environ.get("KEYCLOAK_DJANGO_CLIENT_ID")
        resource_roles = decoded_token.get('resource_access', {}).get(client_name, {}).get('roles', [])
        print(f"Resource Roles: {resource_roles}", flush=True)

    except jwt.DecodeError as e:
        print(f"Error decoding token: {e}", flush=True)
        logout(request)  # Log out the user if token decoding fails
        return redirect("/accounts/oidc/keycloak/login/")


    # check if user has the superuser role in Keycloak
    if("superuser" in resource_roles):
        print(f"User {user.username} has superuser role", flush=True)
        user.is_superuser = True
        user.is_staff = True
    else:
        user.is_superuser = False
        user.is_staff = False
        # Handle user permissions based on the roles in the decoded token
        # permissions are created in web/django_api/aiders/models/__init__.py
        # aiders.auth_permission <-- list of permissions - only the manually created ones are used
        # in Keycloak, the roles are eqivalent with permissions
        for role in resource_roles:
            try:
                permission = Permission.objects.get(codename=role)  # Retrieve the permission by name
                user.user_permissions.add(permission)   # Assign the permission to the user
                print(f"Permission '{role}' assigned to the user.", flush=True)
            except Permission.DoesNotExist:
                print(f"Permission '{role}' does not exist in the database.", flush=True)

        # check if any permissions have been revoked in Keycloak and remove them from the user
        permissions = user.user_permissions.filter(id__gt=260).exclude(codename__regex=r'\d$').order_by('id')
        for permission in permissions:
            if permission.codename not in resource_roles:
                user.user_permissions.remove(permission)
                print(f"Permission '{permission.codename}' removed from the user.", flush=True)

    user.save()


    # aiders.auth_group_permissions <-- assigned to groups (many to many but used as one to one)
    # aiders.auth_group <-- list of groups (their names are the same as the permissions :S)
    # aiders.aiders_user_groups <-- assigned groups to users (many to many)

    # check if it's user's first time login
    if not UserPreferences.objects.filter(user=user).exists():
        UserPreferences.objects.create(use_online_map=True, user=user) # Create default preferences for the user

    # print("\n\n")

    # # Check if the user is the first one to log in and make them a superuser
    # if user.id == 2:  # Adjust this condition as needed
    #     user.is_superuser = True
    #     user.save()

    # # Check if the user has preferences; if not, create them
    # if not UserPreferences.objects.filter(user=user).exists():
    #     UserPreferences.objects.create(use_online_map=True, user=user)
    #     print(f"Preferences created for user {user.id}", flush=True)