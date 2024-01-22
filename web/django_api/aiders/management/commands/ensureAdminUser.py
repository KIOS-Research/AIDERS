from django.contrib.auth import get_user_model
from django.core.management.base import BaseCommand

"""
In Django there are many commands someone can execute. For example, "python manage.py runserver OR python manage.py reset_db etc.
One can create their own Django commands. In this case, this command either checks if a superuser already exists, or it creates one,
and can be used in the following way using Bash: 

 python manage.py ensureAdminUser --username=${USERNAME} \
--email=${EMAIL} \
--password=${PASSWORD} --createIfNotExists

Reference: https://docs.djangoproject.com/en/4.1/howto/custom-management-commands/#module-django.core.management
"""


class Command(BaseCommand):
    help = "Creates an admin user non-interactively if it doesn't exist"

    def add_arguments(self, parser):
        parser.add_argument("--username", help="Admin's username")
        parser.add_argument("--email", help="Admin's email")
        parser.add_argument("--password", help="Admin's password")
        parser.add_argument("--firstname", help="Admin's first name")
        parser.add_argument("--lastname", help="Admin's last name")

        parser.add_argument(
            "--superUserExists", help="Checks if the specified username exists in the database", action="store_true"
        )
        parser.add_argument(
            "--createSuperuser",
            help="Creates a superuser account with the specified details, if it does not already exist",
            action="store_true",
        )

    def handle(self, *args, **options):
        User = get_user_model()
        if options["superUserExists"]:
            superUsers = User.objects.filter(is_superuser=True)
            print(True) if superUsers else print(False)
        elif options["createSuperuser"]:
            if not User.objects.filter(username=options["username"]).exists():
                User.objects.create_superuser(
                    username=options["username"],
                    email=options["email"],
                    password=options["password"],
                    first_name=options["firstname"],
                    last_name=options["lastname"],
                )
                print("Admin User is created!")
