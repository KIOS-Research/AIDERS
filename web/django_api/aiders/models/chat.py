import datetime
from django.contrib.gis.db import models
from django.contrib.auth import get_user_model
from django.utils import timezone

User = get_user_model()


class ChatRoom(models.Model):
    """
    Represents a chat room that users can join to exchange messages
    """
    name = models.CharField(max_length=255)
    description = models.TextField(blank=True, null=True)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE, null=True, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)
    created_by = models.ForeignKey(User, on_delete=models.CASCADE, related_name='created_rooms')
    is_active = models.BooleanField(default=True)
    
    class Meta:
        ordering = ['-created_at']
        constraints = [
            models.UniqueConstraint(
                fields=['operation'], 
                condition=models.Q(operation__isnull=False),
                name='unique_operation_chat_room'
            )
        ]
    
    def __str__(self):
        return self.name
    
    @classmethod
    def get_or_create_for_operation(cls, operation):
        """Get or create a chat room for the given operation"""
        room, created = cls.objects.get_or_create(
            operation=operation,
            defaults={
                'name': f"{operation.operation_name} - Operation Chat",
                'description': f"Operation chat room for {operation.operation_name}",
                'created_by': operation.operator
            }
        )
        return room, created
    
    @property 
    def latest_message(self):
        """Get the latest message in this room"""
        return self.messages.first()
    
    @property
    def message_count(self):
        """Get total number of messages in this room"""
        return self.messages.count()


class ChatMessage(models.Model):
    """
    Represents a message sent in a chat room
    """
    room = models.ForeignKey(ChatRoom, on_delete=models.CASCADE, related_name='messages')
    user = models.ForeignKey(User, on_delete=models.CASCADE, related_name='chat_messages')
    content = models.TextField()
    timestamp = models.DateTimeField(auto_now_add=True)
    is_edited = models.BooleanField(default=False)
    edited_at = models.DateTimeField(null=True, blank=True)
    
    class Meta:
        ordering = ['-timestamp']
    
    def __str__(self):
        return f"{self.user.username}: {self.content[:50]}..."
    
    def edit_message(self, new_content):
        """Edit the message content"""
        self.content = new_content
        self.is_edited = True
        self.edited_at = timezone.now()
        self.save()


class ChatRoomMember(models.Model):
    """
    Represents a user's membership in a chat room
    """
    room = models.ForeignKey(ChatRoom, on_delete=models.CASCADE, related_name='members')
    user = models.ForeignKey(User, on_delete=models.CASCADE, related_name='chat_memberships')
    joined_at = models.DateTimeField(auto_now_add=True)
    last_seen = models.DateTimeField(default=timezone.now)
    is_active = models.BooleanField(default=True)
    
    class Meta:
        unique_together = ('room', 'user')
    
    def __str__(self):
        return f"{self.user.username} in {self.room.name}"
    
    def update_last_seen(self):
        """Update the last seen timestamp"""
        self.last_seen = timezone.now()
        self.save()
