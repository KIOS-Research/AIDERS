from django.contrib.auth import get_user_model
from django.contrib.gis.db import models
from django.db.models import Avg, Count
from django.utils import timezone
from haversine import Direction, Unit, inverse_haversine


class LidarPointSession(models.Model):
    start_time = models.DateTimeField(auto_now_add=True)
    end_time = models.DateTimeField(blank=True, null=True)
    user = models.ForeignKey(get_user_model(), on_delete=models.CASCADE)
    operation = models.ForeignKey("Operation", on_delete=models.CASCADE)
    drone = models.ForeignKey("Drone", on_delete=models.CASCADE)
    is_active = models.BooleanField(default=True)
    is_process = models.BooleanField(default=False)
    path = models.CharField(max_length=255, null=True, blank=True)

    def createNumberOfPointsFieldForSelectedSessions(_sessions):
        return _sessions.annotate(lidar_point_count=Count("lidarpoint"))

    def getNoActiveSessionAndNumberOfLidarPointsByOperationId(_operationId):
        notActiveSessions = LidarPointSession.objects.filter(
            is_active=False, operation_id=_operationId, end_time__isnull=False
        )
        return LidarPointSession.createNumberOfPointsFieldForSelectedSessions(notActiveSessions)

    def getUnprocessedSessionAndNumberOfLidarPointsByOperationId(_operationId):
        notActiveAndNotProcessSessions = LidarPointSession.objects.filter(
            is_active=False, operation_id=_operationId, end_time__isnull=False, is_process=False
        )
        return LidarPointSession.createNumberOfPointsFieldForSelectedSessions(notActiveAndNotProcessSessions)

    def getProcessedSessionAndNumberOfLidarPointsByOperationId(_operationId):
        notActiveAndProcessSessions = LidarPointSession.objects.filter(
            is_active=False, operation_id=_operationId, end_time__isnull=False, is_process=True
        )
        return LidarPointSession.createNumberOfPointsFieldForSelectedSessions(notActiveAndProcessSessions)

    def getLatestActiveSessionByDroneId(_droneId):
        latestActiveSession = (
            LidarPointSession.objects.filter(is_active=True, drone=_droneId).order_by("-start_time").first()
        )
        return latestActiveSession

    def deactivateSession(_session):
        try:
            _session.end_time = timezone.now()
            _session.is_active = False
            _session.save()
            return True
        except LidarPointSession.DoesNotExist:
            return False

    def updateProcessedSessionBySessionIdAndPath(_sessionId, _path):
        session = LidarPointSession.objects.get(id=_sessionId)
        session.is_process = True
        session.path = _path
        session.save()


class LidarPoint(models.Model):
    time = models.DateTimeField(auto_now_add=True)
    x = models.FloatField()
    y = models.FloatField()
    z = models.FloatField()
    intensity = models.FloatField()
    red = models.FloatField()
    green = models.FloatField()
    blue = models.FloatField()
    telemetry = models.ForeignKey("Telemetry", on_delete=models.CASCADE)
    lidar_point_session = models.ForeignKey(LidarPointSession, on_delete=models.CASCADE)

    def getLidarPointsBySessionIdLatestIdAndLimit(_sessionId, _latestPointId, _limitPoints):
        lidarPointsFiltered = LidarPoint.objects.filter(lidar_point_session__id=_sessionId, id__gt=_latestPointId)[
            :_limitPoints
        ]
        lidarPointsTelemetry = lidarPointsFiltered[0] if lidarPointsFiltered else None

        lidarPointList = [
            {
                "id": lidarPointData.id,
                "x": lidarPointData.x,
                "y": lidarPointData.y,
                "z": lidarPointData.z,
                "red": lidarPointData.red,
                "green": lidarPointData.green,
                "blue": lidarPointData.blue,
            }
            for lidarPointData in lidarPointsFiltered
        ]

        return (
            lidarPointList,
            lidarPointsTelemetry.telemetry.lat if lidarPointsTelemetry else None,
            lidarPointsTelemetry.telemetry.lon if lidarPointsTelemetry else None,
            lidarPointsTelemetry.id if lidarPointsTelemetry else None,
        )

    def getAllLidarPointsBySessionId(_sessionId):
        lidarPointsFiltered = LidarPoint.objects.filter(lidar_point_session__id=_sessionId)
        # Convert LidarPoint objects to a list of dictionaries, including only the specified fields
        lidarPointList = [
            {
                "x": lidarPoint.x,
                "y": lidarPoint.y,
                "z": lidarPoint.z,
                "red": lidarPoint.red,
                "green": lidarPoint.green,
                "blue": lidarPoint.blue,
            }
            for lidarPoint in lidarPointsFiltered
        ]
        return lidarPointList

    def getOriginCoordinatesLonLatBySessionId(_sessionId):
        firstLidarPointObject = LidarPoint.objects.filter(lidar_point_session=_sessionId).order_by("time").first()
        return [firstLidarPointObject.telemetry.lon, firstLidarPointObject.telemetry.lat]

    def getLidarMeshDataNeedForVisualizationBySessionId(_sessionId):
        allLidarPointData = LidarPoint.objects.filter(lidar_point_session=_sessionId)

        # Get the first telemetry value
        first_telemetry = allLidarPointData.first().telemetry
        first_session = allLidarPointData.first().lidar_point_session

        avg_x = allLidarPointData.aggregate(Avg("x"))["x__avg"]
        avg_y = allLidarPointData.aggregate(Avg("y"))["y__avg"]

        new_lat_dy, new_lon_dy = inverse_haversine(
            (first_telemetry.lat, first_telemetry.lon), avg_y, Direction.NORTH, unit=Unit.METERS
        )
        new_lat_dx, new_lon_dx = inverse_haversine((new_lat_dy, new_lon_dy), avg_x, Direction.EAST, unit=Unit.METERS)

        dataForVisualization = {
            "latitude": new_lat_dx,
            "longitude": new_lon_dx,
            "path": first_session.path,
        }

        return dataForVisualization
