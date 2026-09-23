from .user import *

from .operation import *

from .detection import *
from .liveStream import *
from .lidar import *

from .weather import *

from .drone import *
from .device import *
from .lora import *
from .staticCamera import *
from .groundVehicle import *
from .droneRid import *
from .adsbAircraft import *

from .flyingReport import *
from .mapPins import *
from .systemMonitoring import *
from .algorithm import *
from .crisisClassification import *
from .pathPlanningOutput import *
from .notification import *
from .chat import *


from .safeDrones import *

from django.contrib.auth.models import Group, Permission
from django.db import connection
from django.contrib.contenttypes.models import ContentType


# LEGACY WAY

# def check_groups():
#     # Create edit_permissions Group
#     try:
#         Group.objects.get(name="edit_permissions")
#     except Exception:
#         create_new_group("edit_permissions", User)
#     # Create create_operations Group
#     try:
#         Group.objects.get(name="create_operations")
#     except Exception:
#         create_new_group("create_operations", Operation)
#     # Create join_operations Group
#     try:
#         Group.objects.get(name="join_operations")
#     except Exception:
#         create_new_group("join_operations", Operation)


# def create_new_group(name, arg1):
#     new_group, created = Group.objects.get_or_create(name=name)
#     if Permission.objects.filter(codename=name).exists():
#         permission = Permission.objects.filter(codename=name)
#     else:
#         ct = ContentType.objects.get_for_model(arg1)
#         permission = Permission.objects.create(codename=name, name=name, content_type=ct)
#     new_group.permissions.add(permission)


# NEW WAY

def init_permissions():
    permissions = [
        # {
        #     "name": "edit_permissions",
        #     "model": User,
        # },
        # {
        #     "name": "create_operations",
        #     "model": Operation,
        # },
        # {
        #     "name": "join_operations",
        #     "model": Operation,
        # }, # delete the above -------------


        {
            "name": "View Operations",
            "codename": "view_operations",
            "model": Operation,
        },        
        {
            "name": "Manage Operations",
            "codename": "manage_operations",
            "model": Operation,
        },
        {
            "name": "Issue Commands",
            "codename": "issue_commands",
            "model": Operation,
        },
        {
            "name": "Manage Users",
            "codename": "manage_users",
            "model": User,
        },               

    ]

    create_permissions(permissions)

    # create_group("admin", permissions, [0, 1, 2])
    # create_group("user", permissions, [1, 2])
    # create_group("viewer", permissions, [2])


def create_permissions(permissions):
    for perm in permissions:
        if not Permission.objects.filter(codename=perm["codename"]).exists():
            ct = ContentType.objects.get_for_model(perm["model"])
            Permission.objects.create(codename=perm["codename"], name=perm["name"], content_type=ct)
            print(f"Permission '{perm['name']}' created for model '{perm['model'].__name__}'", flush=True)



# def create_group(group_name, all_permissions, group_permission_indices):
#     group = None
#     try:
#         group = Group.objects.get(name=group_name)
#     except Exception:
#         group, created = Group.objects.get_or_create(name=group_name)
#         if created:
#             print(f"Group '{group_name}' created", flush=True)


#     # Attach permissions to the group
#     for perm_index in group_permission_indices:
#         permission = Permission.objects.get(codename=all_permissions[perm_index]["name"])
#         group.permissions.add(permission) 


if "auth_group" in connection.introspection.table_names():
    print("+++++++++++++++++++++++++++ init_groups_and_permissions +++++++++++++++++++++++++++", flush=True)
    # check_groups()                  # LEGACY WAY
    init_permissions() # NEW WAY