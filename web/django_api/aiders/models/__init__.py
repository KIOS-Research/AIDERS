from .user import *

from .operation import *

from .detection import *
from .liveStream import *
from .lidar import *

from .weather import *

from .drone import *
from .device import *
from .lora import *

from .flyingReport import *
from .mapPins import *
from .systemMonitoring import *
from .algorithm import *


from .safeDrones import *

from django.contrib.auth.models import Group, Permission
from django.db import connection
from django.contrib.contenttypes.models import ContentType

def check_groups():
    # Create edit_permissions Group
    try:
        Group.objects.get(name="edit_permissions")
    except Exception:
        create_new_group("edit_permissions", User)
    # Create create_operations Group
    try:
        Group.objects.get(name="create_operations")
    except Exception:
        create_new_group("create_operations", Operation)
    # Create join_operations Group
    try:
        Group.objects.get(name="join_operations")
    except Exception:
        create_new_group("join_operations", Operation)


def create_new_group(name, arg1):
    new_group, created = Group.objects.get_or_create(name=name)
    if Permission.objects.filter(codename=name).exists():
        permission = Permission.objects.filter(codename=name)
    else:
        ct = ContentType.objects.get_for_model(arg1)
        permission = Permission.objects.create(codename=name, name=name, content_type=ct)
    new_group.permissions.add(permission)


if "auth_group" in connection.introspection.table_names():
    check_groups()