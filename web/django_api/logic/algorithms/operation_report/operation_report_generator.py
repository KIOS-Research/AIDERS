import os
import json
import hashlib
from datetime import datetime
import math
from io import BytesIO
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection
import numpy as np
try:
    import scipy.stats
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
from reportlab.platypus import (BaseDocTemplate, Flowable, Frame, PageBreak,
                                PageTemplate, Paragraph, Spacer, Table,
                                TableStyle, Image as RLImage, NextPageTemplate)
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.pagesizes import A4, letter
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT, TA_RIGHT
from reportlab.lib.units import inch
from reportlab.lib.colors import HexColor, black, blue, red, green
from reportlab.lib.utils import ImageReader
from django.core.files.storage import default_storage
from django.conf import settings
from django.utils import timezone
from aiders.models import Operation, Drone, Telemetry, Mission, MissionLog, LiveStreamSession, DetectionSession
from aiders.models.detection import DetectedObject

# Add contextily for real map tiles
try:
    import contextily as ctx
    import mercantile
    HAS_CONTEXTILY = True
except ImportError:
    HAS_CONTEXTILY = False

# Tile providers used for report basemaps, in order of preference.
# OpenStreetMap's public tile servers (OpenStreetMap.Mapnik) are deliberately NOT
# used: their usage policy forbids automated/bulk fetching, and instead of failing
# they serve a "403 Access blocked" placeholder image with HTTP 200, which silently
# ends up in the PDF. CartoDB/Esri basemaps allow this kind of use.
DEFAULT_BASEMAP_PROVIDERS = ['CartoDB.Voyager', 'CartoDB.Positron', 'Esri.WorldStreetMap']


def _resolve_tile_provider(dotted_name):
    """Look up a contextily provider such as 'CartoDB.Voyager' by its dotted name"""
    provider = ctx.providers
    for part in dotted_name.split('.'):
        provider = provider[part]
    return provider


def _add_tile_basemap(ax, alpha):
    """Draw a tiled basemap on `ax`, trying each configured provider in turn.

    Returns True if a basemap was drawn, False if every provider failed.
    """
    provider_names = getattr(settings, 'REPORT_BASEMAP_PROVIDERS', DEFAULT_BASEMAP_PROVIDERS)

    for dotted_name in provider_names:
        try:
            source = _resolve_tile_provider(dotted_name)
            ctx.add_basemap(ax,
                            crs='EPSG:4326',  # WGS84 coordinate system
                            source=source,
                            alpha=alpha,
                            zoom='auto',  # Auto-determine zoom level
                            attribution=source.get('attribution'))
            return True
        except Exception as e:
            print(f"Warning: Could not load map tiles from {dotted_name}: {e}")

    return False


styleSheet = getSampleStyleSheet()
page_width, page_height = A4

# Custom styles
title_style = ParagraphStyle(
    name='title',
    fontName='Helvetica-Bold',
    fontSize=18,
    spaceAfter=20,
    alignment=TA_CENTER
)

heading_style = ParagraphStyle(
    name='heading',
    fontName='Helvetica-Bold',
    fontSize=14,
    spaceAfter=12,
    spaceBefore=12,
    alignment=TA_CENTER
)

subheading_style = ParagraphStyle(
    name='subheading',
    fontName='Helvetica-Bold',
    fontSize=12,
    spaceAfter=8,
    spaceBefore=8,
    alignment=TA_CENTER
)

body_style = ParagraphStyle(
    name='body',
    fontName='Helvetica',
    fontSize=10,
    spaceAfter=6
)

caption_style = ParagraphStyle(
    name='caption',
    fontName='Helvetica-Oblique',
    fontSize=9,
    spaceAfter=6,
    alignment=TA_CENTER
)


class CombinedOperationMapFlowable(Flowable):
    """Custom Flowable for rendering combined operation map with all UAV flight paths"""
    
    def __init__(self, operation_data, width=600, height=400, enhanced_maps=True):
        self.operation_data = operation_data
        self.width = width
        self.height = height
        self.enhanced_maps = enhanced_maps
        
    def draw_map(self):
        """Generate a combined map showing all UAV flight paths in the operation"""
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # Collect all telemetry points from all drones
        all_lats = []
        all_lons = []
        drone_paths = []
        
        # Define colors for different drones
        colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown', 'pink', 'gray', 'olive', 'cyan']
        
        for i, drone in enumerate(self.operation_data['drones']):
            if drone['telemetry_points']:
                # Segment telemetry points into continuous flight sessions
                flight_segments = segment_flight_paths(drone['telemetry_points'])
                
                if flight_segments:
                    # Collect all coordinates from all segments for bounds calculation
                    all_segment_lats = []
                    all_segment_lons = []
                    for segment in flight_segments:
                        segment_lats = [point['lat'] for point in segment]
                        segment_lons = [point['lon'] for point in segment]
                        all_segment_lats.extend(segment_lats)
                        all_segment_lons.extend(segment_lons)
                    
                    if all_segment_lats and all_segment_lons:
                        all_lats.extend(all_segment_lats)
                        all_lons.extend(all_segment_lons)
                        drone_paths.append({
                            'name': drone['name'],
                            'segments': flight_segments,
                            'color': colors[i % len(colors)]
                        })
        
        if not all_lats or not all_lons:
            # No telemetry data available for any drone
            ax.text(0.5, 0.5, 'No flight data available for any UAV', 
                   ha='center', va='center', transform=ax.transAxes, fontsize=14)
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
        else:
            # Calculate combined bounds with padding
            lat_padding = max((max(all_lats) - min(all_lats)) * 0.1, 0.002)
            lon_padding = max((max(all_lons) - min(all_lons)) * 0.1, 0.002)
            
            min_lat, max_lat = min(all_lats) - lat_padding, max(all_lats) + lat_padding
            min_lon, max_lon = min(all_lons) - lon_padding, max(all_lons) + lon_padding
            
            # Set map bounds
            ax.set_xlim(min_lon, max_lon)
            ax.set_ylim(min_lat, max_lat)
            
            # Add map background
            self._add_map_background(ax, min_lon, max_lon, min_lat, max_lat, self.enhanced_maps)
            
            # Plot each drone's flight path segments
            for drone_path in drone_paths:
                segments = drone_path['segments']
                color = drone_path['color']
                name = drone_path['name']
                
                # Plot each flight segment separately
                for segment_idx, segment in enumerate(segments):
                    if len(segment) >= 2:  # Only plot segments with at least 2 points
                        segment_lats = [point['lat'] for point in segment]
                        segment_lons = [point['lon'] for point in segment]
                        
                        # Draw the flight path segment
                        label = f'{name} Path' if segment_idx == 0 else None  # Only label first segment to avoid duplicate legend entries
                        ax.plot(segment_lons, segment_lats, color=color, linewidth=2, alpha=0.8, 
                               label=label, zorder=4)
                        
                        # Mark start point for each segment
                        ax.scatter(segment_lons[0], segment_lats[0], c=color, s=100, marker='o', 
                                  alpha=0.9, zorder=5, edgecolors='white', linewidth=2)
                        
                        # Mark end point for each segment
                        if len(segment_lons) > 1:
                            ax.scatter(segment_lons[-1], segment_lats[-1], c=color, s=100, marker='s', 
                                      alpha=0.9, zorder=5, edgecolors='white', linewidth=2)
                        
                        # Add waypoint markers for longer segments
                        # if len(segment_lons) > 10:
                        #     waypoint_interval = max(1, len(segment_lons) // 8)
                        #     waypoint_lons = [segment_lons[i] for i in range(0, len(segment_lons), waypoint_interval)]
                        #     waypoint_lats = [segment_lats[i] for i in range(0, len(segment_lats), waypoint_interval)]
                            
                        #     if len(waypoint_lons) > 2:
                        #         ax.scatter(waypoint_lons[1:-1], waypoint_lats[1:-1], 
                        #                   c=color, s=20, marker='o', alpha=0.6, zorder=4)
            
            # Add operation info box
            total_drones = len(drone_paths)
            active_drones = len([d for d in self.operation_data['drones'] if d['is_connected']])
            
            info_text = f"Operation: {self.operation_data['operation'].operation_name}\n"
            info_text += f"UAVs: {total_drones} total"
            # info_text += f"Total Missions: {self.operation_data['total_missions']}"
            
            ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.9),
                   verticalalignment='top', fontsize=9, zorder=7)
            
            # Add legend (limit to avoid overcrowding)
            if len(drone_paths) <= 8:
                ax.legend(loc='upper right', framealpha=0.8, fontsize=8)
            else:
                # For many drones, show a simplified legend
                ax.text(0.98, 0.02, f"Showing {len(drone_paths)} UAV flight paths", 
                       transform=ax.transAxes, ha='right', va='bottom',
                       bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                       fontsize=8, zorder=7)
        
        # Customize the plot
        ax.set_title(f'Operation {self.operation_data["operation"].operation_name} - Combined Flight Paths', 
                    fontweight='bold', fontsize=14)
        
        # Hide axis labels and tick labels
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_xlabel('')
        ax.set_ylabel('')
        
        # Add coordinate grid
        ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
        ax.set_aspect('equal', adjustable='box')
        
        # Improve layout
        plt.tight_layout()
        
        # Save to BytesIO
        img_buffer = BytesIO()
        plt.savefig(img_buffer, format='png', dpi=200, bbox_inches='tight', 
                   facecolor='white', edgecolor='none')
        img_buffer.seek(0)
        plt.close()
        
        return img_buffer
    
    def _add_map_background(self, ax, min_lon, max_lon, min_lat, max_lat, enhanced_maps=True):
        """Add a real map background from OpenStreetMap or fallback to synthetic background"""
        
        if enhanced_maps and HAS_CONTEXTILY:
            if _add_tile_basemap(ax, alpha=0.8):
                return  # Successfully added real map tiles
            print("Warning: Falling back to synthetic background.")

        # Fallback to synthetic topographic-style background (or if enhanced_maps is False)
        # Create a mesh for the background
        lon_range = np.linspace(min_lon, max_lon, 50)
        lat_range = np.linspace(min_lat, max_lat, 50)
        lon_mesh, lat_mesh = np.meshgrid(lon_range, lat_range)

        # Create a simple terrain-like pattern
        terrain = (np.sin(lon_mesh * 100) * np.cos(lat_mesh * 100) * 0.1 +
                  np.sin(lon_mesh * 200) * np.cos(lat_mesh * 200) * 0.05)
        
        # Add the background with earth-tone colors
        ax.contourf(lon_mesh, lat_mesh, terrain, levels=20, 
                   cmap='terrain', alpha=0.4, zorder=1)
        
        # Add subtle contour lines
        ax.contour(lon_mesh, lat_mesh, terrain, levels=10, 
                  colors='gray', alpha=0.2, linewidths=0.5, zorder=2)
    
    def draw(self):
        """Draw the flowable"""
        img_buffer = self.draw_map()
        
        # Create PIL Image from buffer for ReportLab
        from PIL import Image as PILImage
        img_buffer.seek(0)
        pil_img = PILImage.open(img_buffer)
        
        # Convert to ReportLab Image
        img_buffer_final = BytesIO()
        pil_img.save(img_buffer_final, format='PNG')
        img_buffer_final.seek(0)
        
        # Draw directly on canvas
        from reportlab.lib.utils import ImageReader
        img_reader = ImageReader(img_buffer_final)
        self.canv.drawImage(img_reader, 0, 0, width=self.width, height=self.height)


class FlightPathMapFlowable(Flowable):
    """Custom Flowable for rendering UAV flight path maps"""
    
    def __init__(self, drone_data, width=400, height=300, enhanced_maps=True):
        self.drone_data = drone_data
        self.width = width
        self.height = height
        self.enhanced_maps = enhanced_maps
        
    def draw_map(self):
        """Generate a map showing the drone's flight path with map background"""
        fig, ax = plt.subplots(figsize=(10, 8))
        
        if not self.drone_data['telemetry_points']:
            # No telemetry data available
            ax.text(0.5, 0.5, 'No flight data available', 
                   ha='center', va='center', transform=ax.transAxes, fontsize=14)
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
        else:
            # Segment telemetry points into continuous flight sessions
            flight_segments = segment_flight_paths(self.drone_data['telemetry_points'])
            
            if not flight_segments:
                # No valid telemetry data available
                ax.text(0.5, 0.5, 'No valid GPS data available', 
                       ha='center', va='center', transform=ax.transAxes, fontsize=14)
                ax.set_xlim(0, 1)
                ax.set_ylim(0, 1)
            else:
                # Collect all coordinates from all segments for bounds calculation
                all_lats = []
                all_lons = []
                for segment in flight_segments:
                    segment_lats = [point['lat'] for point in segment]
                    segment_lons = [point['lon'] for point in segment]
                    all_lats.extend(segment_lats)
                    all_lons.extend(segment_lons)
            
            if all_lats and all_lons:
                # Calculate bounds with padding
                lat_padding = max((max(all_lats) - min(all_lats)) * 0.15, 0.002)  # Minimum padding
                lon_padding = max((max(all_lons) - min(all_lons)) * 0.15, 0.002)
                
                min_lat, max_lat = min(all_lats) - lat_padding, max(all_lats) + lat_padding
                min_lon, max_lon = min(all_lons) - lon_padding, max(all_lons) + lon_padding
                
                # Set map bounds
                ax.set_xlim(min_lon, max_lon)
                ax.set_ylim(min_lat, max_lat)
                
                # Add map background (real tiles if enhanced_maps is True)
                self._add_map_background(ax, min_lon, max_lon, min_lat, max_lat, self.enhanced_maps)
                
                # Plot each flight segment separately
                total_distance = 0
                all_points = []
                
                for segment_idx, segment in enumerate(flight_segments):
                    if len(segment) >= 2:  # Only plot segments with at least 2 points
                        segment_lats = [point['lat'] for point in segment]
                        segment_lons = [point['lon'] for point in segment]
                        points = list(zip(segment_lons, segment_lats))
                        all_points.extend(points)
                        
                        # Draw the flight path segment with gradient effect
                        for i in range(len(points) - 1):
                            # Create segments with color gradient (blue to cyan)
                            alpha = 0.8 - (i / len(points)) * 0.3  # Fade effect
                            ax.plot([points[i][0], points[i+1][0]], 
                                   [points[i][1], points[i+1][1]], 
                                   color='blue', linewidth=3, alpha=alpha, zorder=4)
                        
                        # Mark start and end points for each segment
                        ax.scatter(segment_lons[0], segment_lats[0], c='green', s=150, marker='o', 
                                  label='Start' if segment_idx == 0 else None, zorder=6, 
                                  edgecolors='white', linewidth=2)
                        
                        if len(segment_lons) > 1:
                            ax.scatter(segment_lons[-1], segment_lats[-1], c='red', s=150, marker='s', 
                                      label='End' if segment_idx == 0 else None, zorder=6, 
                                      edgecolors='white', linewidth=2)
                        
                        # Add waypoint markers for longer segments
                        # if len(segment_lons) > 10:
                        #     waypoint_interval = max(1, len(segment_lons) // 10)
                        #     waypoint_lons = [segment_lons[i] for i in range(0, len(segment_lons), waypoint_interval)]
                        #     waypoint_lats = [segment_lats[i] for i in range(0, len(segment_lats), waypoint_interval)]
                            
                        #     if len(waypoint_lons) > 2:  # Only show waypoints if we have enough points
                        #         ax.scatter(waypoint_lons[1:-1], waypoint_lats[1:-1], 
                        #                   c='orange', s=30, marker='o', alpha=0.7, 
                        #                   label='Waypoints' if segment_idx == 0 else None, zorder=5, 
                        #                   edgecolors='white', linewidth=1)
                        
                        # Calculate segment distance
                        segment_distance = self._calculate_path_distance(points)
                        total_distance += segment_distance
                
                # Add distance and duration annotations
                duration = self._get_flight_duration()
                
                # Add info box
                info_text = f"Distance: {total_distance:.1f} m"
                if duration:
                    info_text += f"\nDuration: {duration}"
                if len(flight_segments) > 1:
                    info_text += f"\nFlight segments: {len(flight_segments)}"
                
                ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
                       bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
                       verticalalignment='top', fontsize=9, zorder=7)
                
                # Add legend
                ax.legend(loc='upper right', framealpha=0.8)
        
        # Customize the plot
        ax.set_title(f'{self.drone_data["name"]} Flight Path Map', fontweight='bold', fontsize=12)
        
        # Hide axis labels and tick labels
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_xlabel('')
        ax.set_ylabel('')
        
        # Add coordinate grid
        ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
        ax.set_aspect('equal', adjustable='box')
        
        # Improve layout
        plt.tight_layout()
        
        # Save to BytesIO
        img_buffer = BytesIO()
        plt.savefig(img_buffer, format='png', dpi=200, bbox_inches='tight', 
                   facecolor='white', edgecolor='none')
        img_buffer.seek(0)
        plt.close()
        
        return img_buffer
    
    def _add_map_background(self, ax, min_lon, max_lon, min_lat, max_lat, enhanced_maps=True):
        """Add a real map background from OpenStreetMap or fallback to synthetic background"""
        
        if enhanced_maps and HAS_CONTEXTILY:
            if _add_tile_basemap(ax, alpha=0.8):
                return  # Successfully added real map tiles
            print("Warning: Falling back to synthetic background.")

        # Fallback to synthetic topographic-style background (or if enhanced_maps is False)
        # Create a mesh for the background
        lon_range = np.linspace(min_lon, max_lon, 50)
        lat_range = np.linspace(min_lat, max_lat, 50)
        lon_mesh, lat_mesh = np.meshgrid(lon_range, lat_range)
        
        # Create a simple terrain-like pattern
        # This creates a subtle elevation-like pattern
        terrain = (np.sin(lon_mesh * 100) * np.cos(lat_mesh * 100) * 0.1 + 
                  np.sin(lon_mesh * 200) * np.cos(lat_mesh * 200) * 0.05)
        
        # Add the background with earth-tone colors
        im = ax.contourf(lon_mesh, lat_mesh, terrain, levels=20, 
                        cmap='terrain', alpha=0.4, zorder=1)
        
        # Add subtle contour lines
        ax.contour(lon_mesh, lat_mesh, terrain, levels=10, 
                  colors='gray', alpha=0.2, linewidths=0.5, zorder=2)
    
    def _calculate_path_distance(self, points):
        """Calculate total distance of the flight path"""
        total_distance = 0
        for i in range(1, len(points)):
            # Simple distance calculation (not accounting for Earth's curvature)
            lat_diff = points[i][1] - points[i-1][1]
            lon_diff = points[i][0] - points[i-1][0]
            distance = math.sqrt(lat_diff**2 + lon_diff**2) * 111000  # Rough conversion to meters
            total_distance += distance
        return total_distance
    
    def _get_flight_duration(self):
        """Get formatted flight duration from pre-calculated statistics"""
        # Use the flight duration from the statistics (already calculated using LiveStreamSession)
        duration_minutes = self.drone_data.get('statistics', {}).get('flight_duration', 0)
        
        if duration_minutes == 0:
            return None
        
        return format_flight_duration(duration_minutes)
    
    def draw(self):
        """Draw the flowable"""
        img_buffer = self.draw_map()
        
        # Create PIL Image from buffer for ReportLab
        from PIL import Image as PILImage
        img_buffer.seek(0)
        pil_img = PILImage.open(img_buffer)
        
        # Convert to ReportLab Image
        img_buffer_final = BytesIO()
        pil_img.save(img_buffer_final, format='PNG')
        img_buffer_final.seek(0)
        
        # Draw directly on canvas
        from reportlab.lib.utils import ImageReader
        img_reader = ImageReader(img_buffer_final)
        self.canv.drawImage(img_reader, 0, 0, width=self.width, height=self.height)


class DetectionHeatmapFlowable(Flowable):
    """
    Custom Flowable for rendering object detection density heatmap using KDE or hexbin visualization.
    
    Shows a density heatmap of detection locations using only the last position of each unique 
    object (track_id). Uses Gaussian KDE for smooth density visualization with variable opacity 
    layering (higher densities get additional alpha layers for enhanced visibility) and spatial 
    constraint to the actual detection area. Maintains full color spectrum while providing 
    density-based opacity variations. Falls back to hexbin visualization if scipy is not available 
    or there are too few data points.
    """
    
    def __init__(self, operation_data, width=600, height=400, enhanced_maps=True):
        self.operation_data = operation_data
        self.width = width
        self.height = height
        self.enhanced_maps = enhanced_maps
        
    def draw_map(self):
        """
        Generate a detection density heatmap using last position of each unique object.
        
        Uses Gaussian KDE with multi-layer contourf for smooth gradients and natural blob shapes.
        Falls back to hexbin if scipy unavailable. Only shows meaningful detection density areas.
        """
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # Collect all detection coordinates from all drones
        all_detection_coords = []
        detection_counts = {}
        
        for drone in self.operation_data['drones']:
            for detection_session in drone['detection_sessions']:
                # Get all detected objects for this session
                try:
                    session_id = detection_session.get('session_id', None)
                    if session_id:
                        # Get unique track_ids for this session
                        unique_track_ids = DetectedObject.objects.filter(
                            detection_session_id=session_id,
                            track_id__isnull=False
                        ).values_list('track_id', flat=True).distinct()
                        
                        # For each unique track_id, get the latest detection
                        for track_id in unique_track_ids:
                            latest_obj = DetectedObject.objects.filter(
                                detection_session_id=session_id,
                                track_id=track_id
                            ).order_by('-time').first()
                            
                            if latest_obj and latest_obj.lat and latest_obj.lon and is_valid_gps_coordinate(latest_obj.lat, latest_obj.lon):
                                coord_key = (round(latest_obj.lat, 6), round(latest_obj.lon, 6))  # Round to ~1m precision
                                detection_counts[coord_key] = detection_counts.get(coord_key, 0) + 1
                                all_detection_coords.append((latest_obj.lat, latest_obj.lon))
                                
                except Exception as e:
                    print(f"DEBUG: Error getting detected objects for session: {e}")
                    continue
        
        if not all_detection_coords:
            # No detection data available
            ax.text(0.5, 0.5, 'No unique object detection data available for visualization', 
                   ha='center', va='center', transform=ax.transAxes, fontsize=14)
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
        else:
            # Extract coordinates
            lats = [coord[0] for coord in all_detection_coords]
            lons = [coord[1] for coord in all_detection_coords]
            
            # Calculate bounds with padding for good map context
            # lat_padding = max((max(lats) - min(lats)) * 0.1, 0.002)  # Restored to original values
            # lon_padding = max((max(lons) - min(lons)) * 0.1, 0.002)  # Better context around detections
            lat_padding = max((max(lats) - min(lats)) * 0.05, 0.001)  # Restored to original values
            lon_padding = max((max(lons) - min(lons)) * 0.05, 0.001)  # Better context around detections            
            
            min_lat, max_lat = min(lats) - lat_padding, max(lats) + lat_padding
            min_lon, max_lon = min(lons) - lon_padding, max(lons) + lon_padding
            
            # Set map bounds
            ax.set_xlim(min_lon, max_lon)
            ax.set_ylim(min_lat, max_lat)
            
            # Add map background
            self._add_map_background(ax, min_lon, max_lon, min_lat, max_lat, self.enhanced_maps)
            
            # Create a proper heatmap using gaussian kernel density estimation
            # Use a custom colormap that goes from orange -> red -> purple for better contrast
            # against typical map backgrounds (which often have pale yellow, blue, green, brown)
            from matplotlib.colors import LinearSegmentedColormap
            
            colors = ['yellow', 'orange', 'red']
            n_bins = 100
            cmap = LinearSegmentedColormap.from_list('detection_heatmap', colors, N=n_bins)
            
            # Try to create a proper density heatmap if scipy is available
            if HAS_SCIPY and len(all_detection_coords) > 3:
                try:
                    import scipy.stats as stats
                    
                    # Use the full map bounds for KDE calculation to avoid sharp edges
                    # This allows the heatmap to extend smoothly to the map boundaries
                    kde_min_lat = min_lat
                    kde_max_lat = max_lat
                    kde_min_lon = min_lon
                    kde_max_lon = max_lon
                    
                    # Create a high-resolution mesh grid to avoid pixelated edges
                    # Use much higher resolution for smooth, non-pixelated appearance
                    lon_range = np.linspace(kde_min_lon, kde_max_lon, 300)  # Increased from 100 to 300
                    lat_range = np.linspace(kde_min_lat, kde_max_lat, 300)  # Increased from 100 to 300
                    lon_mesh, lat_mesh = np.meshgrid(lon_range, lat_range)
                    
                    # Create positions for kde
                    positions = np.vstack([lon_mesh.ravel(), lat_mesh.ravel()])
                    
                    # Prepare detection data for kde
                    detection_lons = np.array(lons)
                    detection_lats = np.array(lats)
                    
                    # Apply weights based on detection counts at each location
                    weights = []
                    for coord in all_detection_coords:
                        coord_key = (round(coord[0], 6), round(coord[1], 6))
                        weights.append(detection_counts.get(coord_key, 1))
                    weights = np.array(weights)
                    
                    # Create kernel density estimation
                    data_points = np.vstack([detection_lons, detection_lats])
                    kde = stats.gaussian_kde(data_points, weights=weights)
                    
                    # Evaluate kde on the mesh
                    density = kde(positions).T
                    density = density.reshape(lon_mesh.shape)
                    
                    # Apply proper normalization to fix extreme color scale values
                    # Clip extreme outliers and normalize to reasonable scale
                    density_99th = np.percentile(density, 99)  # Use 99th percentile as max to clip outliers
                    density_clipped = np.clip(density, 0, density_99th)
                    
                    # Normalize to 0-1 scale for proper color mapping
                    if np.max(density_clipped) > 0:
                        density_normalized = density_clipped / np.max(density_clipped)
                    else:
                        density_normalized = density_clipped
                    
                    # Apply an aggressive threshold first to reduce low-density spread
                    # Use a much higher percentile to focus on meaningful detection areas
                    pre_threshold = np.percentile(density_normalized[density_normalized > 0], 65) if np.any(density_normalized > 0) else 0
                    
                    # Create a distance-based mask that focuses heatmap around actual detections
                    # This reduces the area covered by low-density regions
                    detection_coords = np.column_stack([detection_lats, detection_lons])
                    
                    # Create a distance mask that fades based on distance from actual detections
                    max_influence_distance = 0.0006  # Reduced from 0.002 to ~60m influence radius for smaller areas
                    distance_mask = np.zeros_like(lat_mesh)
                    
                    for det_lat, det_lon in detection_coords:
                        # Calculate distance from this detection to all grid points
                        lat_diff = lat_mesh - det_lat
                        lon_diff = lon_mesh - det_lon
                        distance = np.sqrt(lat_diff**2 + lon_diff**2)
                        
                        # Create influence mask for this detection (inverse distance)
                        influence = np.maximum(0, 1 - distance / max_influence_distance)
                        distance_mask = np.maximum(distance_mask, influence)
                    
                    # Apply the distance mask to constrain the heatmap to areas near actual detections
                    density_constrained = density_normalized * distance_mask
                    
                    # Create a soft fade mask to smooth edges at map boundaries (smaller fade area)
                    fade_distance = 0.1  # Reduced from 0.15 to make fade area smaller
                    
                    # Calculate fade distances
                    lat_fade_dist = (kde_max_lat - kde_min_lat) * fade_distance
                    lon_fade_dist = (kde_max_lon - kde_min_lon) * fade_distance
                    
                    # Create fade masks for each edge
                    lat_fade_mask = np.ones_like(lat_mesh)
                    lon_fade_mask = np.ones_like(lon_mesh)
                    
                    # Bottom edge fade
                    bottom_mask = lat_mesh < (kde_min_lat + lat_fade_dist)
                    lat_fade_mask[bottom_mask] = (lat_mesh[bottom_mask] - kde_min_lat) / lat_fade_dist
                    
                    # Top edge fade
                    top_mask = lat_mesh > (kde_max_lat - lat_fade_dist)
                    lat_fade_mask[top_mask] = (kde_max_lat - lat_mesh[top_mask]) / lat_fade_dist
                    
                    # Left edge fade
                    left_mask = lon_mesh < (kde_min_lon + lon_fade_dist)
                    lon_fade_mask[left_mask] = (lon_mesh[left_mask] - kde_min_lon) / lon_fade_dist
                    
                    # Right edge fade
                    right_mask = lon_mesh > (kde_max_lon - lon_fade_dist)
                    lon_fade_mask[right_mask] = (kde_max_lon - lon_mesh[right_mask]) / lon_fade_dist
                    
                    # Combine fade masks (use minimum to ensure corners fade properly)
                    combined_fade_mask = np.minimum(lat_fade_mask, lon_fade_mask)
                    combined_fade_mask = np.clip(combined_fade_mask, 0, 1)
                    
                    # Apply both the distance constraint and fade mask
                    density_with_smooth_edges = density_constrained * combined_fade_mask
                    
                    # Apply power scaling to emphasize high-density areas and suppress low-density areas
                    # This makes the heatmap more focused on significant detection areas
                    power_factor = 2.0  # Higher values make the effect more dramatic
                    density_with_smooth_edges = np.power(density_with_smooth_edges, power_factor)
                    
                    # Apply gaussian smoothing to eliminate pixelated edges
                    # This creates truly smooth boundaries regardless of grid resolution
                    try:
                        from scipy import ndimage
                        # Apply stronger gaussian smoothing for ultra-smooth edges
                        sigma = 2.0  # Increased smoothing for even smoother edges
                        density_with_smooth_edges = ndimage.gaussian_filter(density_with_smooth_edges, sigma=sigma)
                        print(f"DEBUG: Applied gaussian smoothing with sigma={sigma} for ultra-smooth edges")
                    except ImportError:
                        print("DEBUG: scipy not available, skipping gaussian smoothing")
                        pass
                    
                    # Apply a much higher threshold to focus on smaller, more significant areas
                    # Use a much higher percentile to reduce the heatmap coverage area
                    threshold = np.percentile(density_with_smooth_edges[density_with_smooth_edges > 0], 80) if np.any(density_with_smooth_edges > 0) else 0
                    
                    # Create a proper mask that completely removes low-density background areas
                    # Only keep areas that are significantly above the background noise
                    significant_density_mask = density_with_smooth_edges > threshold
                    
                    # Instead of hard binary masking, create a smooth falloff near the threshold
                    # This eliminates sharp edges while keeping the same overall coverage
                    density_masked = density_with_smooth_edges.copy()
                    
                    # Create a narrower smooth transition zone to maintain smaller heatmap area
                    # This keeps the area focused while ensuring smooth edges
                    transition_width = threshold * 0.02  # Reduced from 0.1 to 0.05 for tighter area
                    lower_threshold = threshold - transition_width
                    
                    # Apply smooth falloff in the transition zone (sigmoid-like curve)
                    transition_mask = (density_with_smooth_edges >= lower_threshold) & (density_with_smooth_edges < threshold)
                    if np.any(transition_mask):
                        transition_values = density_with_smooth_edges[transition_mask]
                        # Normalize to 0-1 in transition zone
                        normalized = (transition_values - lower_threshold) / transition_width
                        # Apply smooth sigmoid falloff
                        smooth_factor = 1 / (1 + np.exp(-6 * (normalized - 0.5)))  # Smooth S-curve
                        density_masked[transition_mask] = transition_values * smooth_factor
                    
                    # Areas well below threshold become NaN
                    density_masked[density_with_smooth_edges < lower_threshold] = np.nan
                    
                    # Use imshow with standard bilinear interpolation
                    # The smooth threshold transition above handles the edge smoothing
                    heatmap = ax.imshow(density_masked, extent=[kde_min_lon, kde_max_lon, kde_min_lat, kde_max_lat], 
                                       cmap=cmap, origin='lower', alpha=0.6, zorder=5,
                                       vmin=lower_threshold, vmax=np.nanmax(density_masked),
                                       interpolation='bilinear', aspect='auto')
                    
                    # Optional: Add a second layer with higher alpha for hotspots to enhance visibility
                    if np.nanmax(density_masked) > threshold * 2:
                        high_density_threshold = np.nanpercentile(density_masked[~np.isnan(density_masked)], 80)
                        high_density_masked = np.where(density_masked > high_density_threshold, density_masked, np.nan)
                        ax.imshow(high_density_masked, extent=[kde_min_lon, kde_max_lon, kde_min_lat, kde_max_lat], 
                                 cmap=cmap, origin='lower', alpha=0.3, zorder=6,
                                 vmin=high_density_threshold, vmax=np.nanmax(density_masked),
                                 interpolation='bilinear', aspect='auto')
                    
                    # Create a dummy mappable for the colorbar (without alpha)
                    from matplotlib import cm
                    import matplotlib.colors as mcolors
                    norm = mcolors.Normalize(vmin=threshold, vmax=np.nanmax(density_masked))
                    mappable = cm.ScalarMappable(norm=norm, cmap=cmap)
                    
                    # Add colorbar with normalized scale
                    # cbar = plt.colorbar(mappable, ax=ax, label='Detection Density', shrink=0.8)
                    # # hide the colorbar ticks for cleaner look
                    # cbar.ax.tick_params(labelsize=8)
                    # cbar.set_ticks([])  # Hide ticks for cleaner look

                    
                    print(f"DEBUG: KDE grid: lat {kde_min_lat:.6f}-{kde_max_lat:.6f}, lon {kde_min_lon:.6f}-{kde_max_lon:.6f}")
                    print(f"DEBUG: Map bounds: lat {min_lat:.6f}-{max_lat:.6f}, lon {min_lon:.6f}-{max_lon:.6f}")
                    print(f"DEBUG: Created smooth imshow KDE heatmap with reduced detection areas. Density range: {threshold:.3f}-{np.nanmax(density_masked):.3f}, Threshold: {threshold:.3f}")
                    
                except Exception as e:
                    print(f"DEBUG: Could not create KDE heatmap: {e}. Falling back to hexbin.")
                    # Fallback to hexbin if KDE fails
                    hb = ax.hexbin(lons, lats, gridsize=12, cmap=cmap, alpha=0.5, 
                                  mincnt=1, zorder=5, edgecolors='none')
                    
                    # Add colorbar
                    cbar = plt.colorbar(hb, ax=ax, label='Detection Density', shrink=0.8)
                    cbar.ax.tick_params(labelsize=8)
            else:
                # Fallback to hexbin for better heatmap effect than individual points
                print("DEBUG: Using hexbin heatmap (scipy not available or too few points)")
                hb = ax.hexbin(lons, lats, gridsize=12, cmap=cmap, alpha=0.7, 
                              mincnt=1, zorder=5, edgecolors='none')
                
                # Add colorbar
                cbar = plt.colorbar(hb, ax=ax, label='Detection Density', shrink=0.8)
                cbar.ax.tick_params(labelsize=8)
            
            # Add small marker points to show exact detection locations (optional, smaller and more subtle)
            unique_coords = list(detection_counts.keys())
            if len(unique_coords) <= 30:  # Only show points if not too many
                for coord, count in detection_counts.items():
                    lat, lon = coord
                    # Much smaller points that complement the heatmap
                    size = min(12 + count * 2, 25)  # Smaller scaling
                    
                    ax.scatter(lon, lat, c='white', s=size, alpha=0.9, 
                              edgecolors='black', linewidth=1, zorder=7)  # White dots with black edges
            
            # Add statistics info box
            total_unique_objects = len(all_detection_coords)
            unique_locations = len(detection_counts)
            max_detections_at_location = max(detection_counts.values()) if detection_counts else 0
            
            info_text = f"Unique Objects: {total_unique_objects}"
            # info_text += f"Detection Hotspots: {unique_locations}"
            
            ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.9),
                   verticalalignment='top', fontsize=9, zorder=7)
        
        # Customize the plot
        ax.set_title(f'Object Detection Heatmap - Operation {self.operation_data["operation"].operation_name}', 
                    fontweight='bold', fontsize=14)
        
        # Hide axis labels and tick labels
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_xlabel('')
        ax.set_ylabel('')
        
        # Add coordinate grid
        ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
        ax.set_aspect('equal', adjustable='box')
        
        # Improve layout
        plt.tight_layout()
        
        # Save to BytesIO
        img_buffer = BytesIO()
        plt.savefig(img_buffer, format='png', dpi=200, bbox_inches='tight', 
                   facecolor='white', edgecolor='none')
        img_buffer.seek(0)
        plt.close()
        
        return img_buffer
    
    def _add_map_background(self, ax, min_lon, max_lon, min_lat, max_lat, enhanced_maps=True):
        """Add a real map background from OpenStreetMap or fallback to synthetic background"""
        
        if enhanced_maps and HAS_CONTEXTILY:
            # More transparent than other maps, for heatmap visibility
            if _add_tile_basemap(ax, alpha=0.6):
                return  # Successfully added real map tiles
            print("Warning: Falling back to synthetic background.")

        # Fallback to synthetic topographic-style background
        lon_range = np.linspace(min_lon, max_lon, 50)
        lat_range = np.linspace(min_lat, max_lat, 50)
        lon_mesh, lat_mesh = np.meshgrid(lon_range, lat_range)
        
        # Create a simple terrain-like pattern
        terrain = (np.sin(lon_mesh * 100) * np.cos(lat_mesh * 100) * 0.1 + 
                  np.sin(lon_mesh * 200) * np.cos(lat_mesh * 200) * 0.05)
        
        # Add the background with earth-tone colors (more transparent for heatmap)
        ax.contourf(lon_mesh, lat_mesh, terrain, levels=20, 
                   cmap='terrain', alpha=0.3, zorder=1)
        
        # Add subtle contour lines
        ax.contour(lon_mesh, lat_mesh, terrain, levels=10, 
                  colors='gray', alpha=0.2, linewidths=0.5, zorder=2)
    
    def draw(self):
        """Draw the flowable"""
        img_buffer = self.draw_map()
        
        # Create PIL Image from buffer for ReportLab
        from PIL import Image as PILImage
        img_buffer.seek(0)
        pil_img = PILImage.open(img_buffer)
        
        # Convert to ReportLab Image
        img_buffer_final = BytesIO()
        pil_img.save(img_buffer_final, format='PNG')
        img_buffer_final.seek(0)
        
        # Draw directly on canvas
        from reportlab.lib.utils import ImageReader
        img_reader = ImageReader(img_buffer_final)
        self.canv.drawImage(img_reader, 0, 0, width=self.width, height=self.height)


def calculate_flight_statistics(telemetry_points, drone=None, start_time=None, end_time=None):
    """
    Calculate flight statistics from telemetry data
    
    Args:
        telemetry_points: List of telemetry data points
        drone: Drone model instance (optional, for accurate flight duration and distance calculation)
        start_time: Start time filter for flight duration calculation
        end_time: End time filter for flight duration calculation
    
    Returns:
        dict: Dictionary containing flight statistics including accurate flight duration and distance
    """
    if not telemetry_points:
        return {
            'total_distance': 0,
            'max_altitude': 0,
            'max_speed': 0,
            'flight_duration': 0,
            'avg_speed': 0
        }
    
    # Filter out invalid GPS coordinates (0,0 before GPS lock)
    valid_points = filter_valid_telemetry_points(telemetry_points)
    
    # Calculate distance - use session-based calculation if drone is provided
    total_distance = 0
    if drone:
        # Get session-based accurate distance calculation
        session_data = get_drone_livestream_sessions(drone, start_time, end_time)
        total_distance = sum(s['distance_meters'] for s in session_data)
        
        # If session-based calculation returns 0 but we have valid telemetry points, use telemetry fallback
        if total_distance == 0 and len(valid_points) >= 2:
            print(f"DEBUG: Session-based distance is 0 for drone {drone.drone_name}, using telemetry fallback. Sessions: {len(session_data)}, Valid points: {len(valid_points)}")
            for i in range(1, len(valid_points)):
                prev_point = valid_points[i-1]
                curr_point = valid_points[i]
                # Simple distance calculation (not accounting for Earth's curvature)
                lat_diff = curr_point['lat'] - prev_point['lat']
                lon_diff = curr_point['lon'] - prev_point['lon']
                distance = math.sqrt(lat_diff**2 + lon_diff**2) * 111000  # Rough conversion to meters
                total_distance += distance
            print(f"DEBUG: Telemetry fallback distance for {drone.drone_name}: {total_distance:.2f}m")
    else:
        # Fallback to telemetry-based calculation using only valid points
        if len(valid_points) >= 2:
            for i in range(1, len(valid_points)):
                prev_point = valid_points[i-1]
                curr_point = valid_points[i]
                # Simple distance calculation (not accounting for Earth's curvature)
                lat_diff = curr_point['lat'] - prev_point['lat']
                lon_diff = curr_point['lon'] - prev_point['lon']
                distance = math.sqrt(lat_diff**2 + lon_diff**2) * 111000  # Rough conversion to meters
                total_distance += distance
    
    max_altitude = 0
    max_speed = 0
    speeds = []
    
    for i, point in enumerate(telemetry_points):
        # Max altitude
        if point['alt'] > max_altitude:
            max_altitude = point['alt']
        
        # Speed
        if point.get('velocity'):
            speeds.append(point['velocity'])
            if point['velocity'] > max_speed:
                max_speed = point['velocity']
    
    # Flight duration - use LiveStreamSession data for accurate calculation
    # This accounts for disconnection periods and only counts active flight time
    if drone:
        flight_duration = calculate_actual_flight_duration(drone, start_time, end_time)
        
        # If session-based flight duration is 0 but we have telemetry points, use telemetry fallback
        if flight_duration == 0 and len(telemetry_points) > 1:
            print(f"DEBUG: Session-based flight duration is 0 for drone {drone.drone_name}, using telemetry fallback")
            start_time_telem = telemetry_points[0]['time']
            end_time_telem = telemetry_points[-1]['time']
            if isinstance(start_time_telem, str):
                start_time_telem = datetime.fromisoformat(start_time_telem.replace('Z', '+00:00'))
            if isinstance(end_time_telem, str):
                end_time_telem = datetime.fromisoformat(end_time_telem.replace('Z', '+00:00'))
            
            flight_duration = (end_time_telem - start_time_telem).total_seconds() / 60  # minutes
            print(f"DEBUG: Telemetry fallback flight duration for {drone.drone_name}: {flight_duration:.2f} minutes")
    else:
        # Fallback to old method if drone is not provided (may include disconnection periods)
        if len(telemetry_points) > 1:
            start_time_telem = telemetry_points[0]['time']
            end_time_telem = telemetry_points[-1]['time']
            if isinstance(start_time_telem, str):
                start_time_telem = datetime.fromisoformat(start_time_telem.replace('Z', '+00:00'))
            if isinstance(end_time_telem, str):
                end_time_telem = datetime.fromisoformat(end_time_telem.replace('Z', '+00:00'))
            
            flight_duration = (end_time_telem - start_time_telem).total_seconds() / 60  # minutes
        else:
            flight_duration = 0
    
    avg_speed = sum(speeds) / len(speeds) if speeds else 0
    
    return {
        'total_distance': round(total_distance, 2),
        'max_altitude': round(max_altitude, 2),
        'max_speed': round(max_speed, 2),
        'flight_duration': round(flight_duration, 2),
        'avg_speed': round(avg_speed, 2)
    }


def get_operation_data(operation_name, start_time=None, end_time=None):
    """
    Gather all operation data including drones and telemetry
    
    Args:
        operation_name: Name of the operation
        start_time: Optional start time filter. If None, uses operation.created_at
        end_time: Optional end time filter. If None, uses operation.ended_at (if not null)
    
    Returns:
        dict: Operation data including filtered telemetry, sessions, and missions
    """
    try:
        operation = Operation.objects.get(operation_name=operation_name)
    except Operation.DoesNotExist:
        raise ValueError(f"Operation '{operation_name}' not found")
    
    # Use operation's own start and end dates if no explicit time range is provided
    # This ensures the report only includes data from the operation's actual duration
    if start_time is None:
        start_time = operation.created_at
    if end_time is None and operation.ended_at is not None:
        end_time = operation.ended_at
    
    # Get drones for this operation
    drones = Drone.objects.filter(operation=operation)
    drone_ids = list(drones.values_list('id', flat=True))
    
    # Get total missions for the operation that belong to drones currently in the operation
    # First get all missions for the operation within the operation's time range
    total_missions_filter = {'operation': operation}
    # Apply operation's time range for total count
    total_missions_filter['time__gte'] = start_time
    if end_time:
        total_missions_filter['time__lte'] = end_time
    
    # Then filter by missions that have mission logs with drones currently in the operation
    mission_logs_with_current_drones = MissionLog.objects.filter(
        operation=operation,
        drone__id__in=drone_ids,
        action="START_MISSION",
        mission__time__gte=start_time
    )
    if end_time:
        mission_logs_with_current_drones = mission_logs_with_current_drones.filter(
            mission__time__lte=end_time
        )
    
    mission_ids_for_total = mission_logs_with_current_drones.values_list('mission_id', flat=True)
    total_operation_missions = Mission.objects.filter(
        id__in=mission_ids_for_total
    ).count()
    
    # Get missions for the operation that belong to drones currently in the operation
    missions_filter = {'operation': operation}
    missions_filter['time__gte'] = start_time
    if end_time:
        missions_filter['time__lte'] = end_time
    
    # Filter missions by those that have mission logs with drones currently in the operation
    mission_logs_with_current_drones_filtered = MissionLog.objects.filter(
        operation=operation,
        drone__id__in=drone_ids,
        action="START_MISSION",
        mission__time__gte=start_time
    )
    
    if end_time:
        mission_logs_with_current_drones_filtered = mission_logs_with_current_drones_filtered.filter(
            mission__time__lte=end_time
        )
    
    filtered_mission_ids = mission_logs_with_current_drones_filtered.values_list('mission_id', flat=True)
    
    # Get missions ordered by time ascending
    operation_missions = Mission.objects.filter(
        id__in=filtered_mission_ids
    ).order_by('time')
    
    operation_data = {
        'operation': operation,
        'drones': [],
        'missions': operation_missions,
        'total_drones': drones.count(),
        'active_drones': drones.filter(is_connected_with_platform=True).count(),
        'total_missions': total_operation_missions,
        'start_time': start_time,  # Use the actual start_time (operation.created_at if none provided)
        'end_time': end_time  # Use the actual end_time (operation.ended_at if none provided and not null)
    }
    
    for drone in drones:
        # Get telemetry data within operation's time range
        telemetry_filter = {'drone': drone}
        telemetry_filter['time__gte'] = start_time
        if end_time:
            telemetry_filter['time__lte'] = end_time
            
        telemetry_data = Telemetry.objects.filter(**telemetry_filter).order_by('time')
        
        # Convert telemetry to list of dicts
        telemetry_points = []
        for t in telemetry_data:
            telemetry_points.append({
                'time': t.time,
                'lat': t.lat,
                'lon': t.lon,
                'alt': t.alt,
                'velocity': t.velocity,
                'heading': t.heading,
                'battery_percentage': t.battery_percentage,
                'drone_state': t.drone_state
            })
        
        # Note: We store all telemetry points but filtering for invalid GPS coordinates 
        # (Lat:0, Long:0) is handled in individual functions that process the data
        
        # Calculate statistics (this function handles GPS filtering internally)
        stats = calculate_flight_statistics(telemetry_points, drone=drone, start_time=start_time, end_time=end_time)
        
        # Get LiveStreamSession data for this drone
        session_data = get_drone_livestream_sessions(drone, start_time, end_time)
        
        # Get DetectionSession data for this drone
        detection_session_data = get_drone_detection_sessions(drone, start_time, end_time)
        
        # Get drone-specific mission information
        drone_mission_logs = MissionLog.objects.filter(drone=drone, operation=operation, action="START_MISSION").order_by('-time')
        
        drone_data = {
            'name': drone.drone_name,
            'model': drone.model,
            'type': drone.type,
            'configuration': drone.configuration,
            'is_connected': drone.is_connected_with_platform,
            'telemetry_points': telemetry_points,
            'statistics': stats,
            'sessions': session_data,
            'detection_sessions': detection_session_data,
            'missions': drone_mission_logs.count(),  # Drone-specific missions
            'total_mission_logs': drone_mission_logs.count()
        }
        
        operation_data['drones'].append(drone_data)
    
    return operation_data


def create_operation_summary(story, operation_data, enhanced_maps=True):
    """Create the operation summary section"""
    operation = operation_data['operation']
    
    # Title (smaller since we have logo in header)
    # story.append(Paragraph("Operation Report", heading_style))
    # story.append(Spacer(1, 1))
    
    # Operation details
    # story.append(Paragraph("Operation Details", heading_style))
    
    # Calculate operation totals for the details table
    total_flight_time = sum(drone['statistics']['flight_duration'] for drone in operation_data['drones'])
    total_distance = sum(drone['statistics']['total_distance'] for drone in operation_data['drones'])
    
    # Calculate detection session totals for operation summary
    total_detection_sessions = sum(len(drone['detection_sessions']) for drone in operation_data['drones'])
    total_detection_time = sum(
        sum(s['duration_minutes'] for s in drone['detection_sessions']) 
        for drone in operation_data['drones']
    )
    
    # Format the reported period for display. These are the dates selected by the user,
    # falling back to the operation's own start/end when nothing was selected.
    start_time = operation_data.get('start_time', operation.created_at)
    end_time = operation_data.get('end_time', operation.ended_at)

    def format_report_datetime(value):
        if timezone.is_aware(value):
            value = timezone.localtime(value)
        return value.strftime('%Y-%m-%d %H:%M')

    if start_time and end_time:
        data_range = f"{format_report_datetime(start_time)} - {format_report_datetime(end_time)} UTC"
    elif start_time:
        data_range = f"From {format_report_datetime(start_time)} UTC (ongoing)"
    else:
        data_range = "All available data"

    details_data = [
        ['Operation Name:', operation.operation_name],
        ['Location:', operation.location],
        ['Description:', operation.description or 'N/A'],
        ['Started:', operation.created_at.strftime('%Y-%m-%d %H:%M UTC')],
        ['Ended:', operation.ended_at.strftime('%Y-%m-%d %H:%M UTC') if operation.ended_at else '-'],
        ['Report Period:', data_range],
        ['Operator:', str(operation.operator)],
        # ['Status:', 'Active' if operation.active else 'Completed'],
        ['Total UAVs:', str(operation_data['total_drones'])],
        # ['Active UAVs:', str(operation_data['active_drones'])],
        # ['Missions:', str(operation_data['total_missions'])],
        ['Total Flight Time:', format_flight_duration(total_flight_time)],
        ['Total Distance:', format_distance(total_distance)],
        # ['Detection Sessions:', str(total_detection_sessions)],
        # ['Total Detection Time:', format_flight_duration(total_detection_time)],
    ]
    
    
    # Create table
    details_table = Table(details_data, colWidths=[2*inch, 4*inch])
    details_table.setStyle(TableStyle([
        ('FONTNAME', (0, 0), (-1, -1), 'Helvetica'),
        ('FONTSIZE', (0, 0), (-1, -1), 10),
        ('FONTNAME', (0, 0), (0, -1), 'Helvetica-Bold'),
        ('ALIGN', (0, 0), (0, -1), 'RIGHT'),
        ('ALIGN', (1, 0), (1, -1), 'LEFT'),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('LEFTPADDING', (0, 0), (-1, -1), 6),
        ('RIGHTPADDING', (0, 0), (-1, -1), 6),
        ('TOPPADDING', (0, 0), (-1, -1), 3),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 3),
        ('GRID', (0, 0), (-1, -1), 0.5, black),
    ]))
    
    story.append(details_table)
    story.append(Spacer(1, 15))


    # UAV Summary
    story.append(Paragraph("UAV Summary", heading_style))
    
    if not operation_data['drones']:
        story.append(Paragraph("No UAVs found for this operation.", body_style))
        return
    
    # Summary table
    summary_data = [['UAV Name', 'Model', 'Type', 'Flight Time', 'Distance', 'Max Alt (m)']]
    
    for drone in operation_data['drones']:
        status = 'Connected' if drone['is_connected'] else 'Disconnected'
        summary_data.append([
            drone['name'],
            drone['model'][:16] or 'N/A', # max 16 characters for model
            drone['type'] or 'N/A',
            # status,
            # str(drone['missions']),
            format_flight_duration(drone['statistics']['flight_duration']),
            format_distance(drone['statistics']['total_distance']),  # Use formatted distance
            str(drone['statistics']['max_altitude'])
        ])
    
    summary_table = Table(summary_data, colWidths=[1.2*inch, 1.2*inch, 0.7*inch, 1.1*inch, 1.1*inch, 0.8*inch])
    summary_table.setStyle(TableStyle([
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTNAME', (0, 1), (-1, -1), 'Helvetica'),
        ('FONTSIZE', (0, 0), (-1, -1), 9),
        ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('GRID', (0, 0), (-1, -1), 0.5, black),
        ('BACKGROUND', (0, 0), (-1, 0), HexColor('#E0E0E0')),
    ]))
    
    story.append(summary_table)




    story.append(PageBreak())
    
    # operation charts

    # Add negative spacer to reduce top margin on new page
    story.append(Spacer(1, -130))  # Negative spacer to move content up
    
    # Combined Operation Map - showing all UAV flight paths
    # story.append(Paragraph("Operation Overview Map", subheading_style))
    
    # Check if we have any flight data
    has_flight_data = any(drone['telemetry_points'] for drone in operation_data['drones'])
    
    if has_flight_data:
        # Create the combined operation map (reduced height to save space)
        combined_map = CombinedOperationMapFlowable(operation_data, width=6.3*inch, height=4.5*inch, enhanced_maps=enhanced_maps)
        story.append(combined_map)
        # story.append(Spacer(1, 0))
        story.append(Paragraph(f"Combined flight paths for all UAVs in operation {operation_data['operation'].operation_name}", caption_style))
    else:
        story.append(Paragraph("No flight data available for visualization.", body_style))
    
    story.append(Spacer(1, 20))
    
    # Detection Heatmap - showing all detection locations
    # Check if we have any detection data
    has_detection_data = any(drone['detection_sessions'] for drone in operation_data['drones'])
    
    if has_detection_data:
        # Create the detection heatmap
        detection_heatmap = DetectionHeatmapFlowable(operation_data, width=6.3*inch, height=4.5*inch, enhanced_maps=enhanced_maps)
        story.append(detection_heatmap)
        story.append(Spacer(1, 0))
        story.append(Paragraph(f"Unique object locations heatmap showing final positions of detected objects during operation {operation_data['operation'].operation_name}", caption_style))
    else:
        story.append(Paragraph("No object detection data available for heatmap visualization.", body_style))
    
    # story.append(Spacer(1, 8))


def create_drones_summary(story, operation_data):
    """Create the drones summary section"""
    story.append(Paragraph("UAV Summary", heading_style))
    
    if not operation_data['drones']:
        story.append(Paragraph("No UAVs found for this operation.", body_style))
        return
    
    # Summary table
    summary_data = [['UAV Name', 'Model', 'Type', 'Flight Time', 'Distance', 'Max Alt (m)']]
    
    for drone in operation_data['drones']:
        status = 'Connected' if drone['is_connected'] else 'Disconnected'
        summary_data.append([
            drone['name'],
            drone['model'] or 'N/A',
            drone['type'] or 'N/A',
            # status,
            # str(drone['missions']),
            format_flight_duration(drone['statistics']['flight_duration']),
            format_distance(drone['statistics']['total_distance']),  # Use formatted distance
            str(drone['statistics']['max_altitude'])
        ])
    
    summary_table = Table(summary_data, colWidths=[1.2*inch, 1.2*inch, 0.7*inch, 1.1*inch, 1.1*inch, 0.8*inch])
    summary_table.setStyle(TableStyle([
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTNAME', (0, 1), (-1, -1), 'Helvetica'),
        ('FONTSIZE', (0, 0), (-1, -1), 9),
        ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('GRID', (0, 0), (-1, -1), 0.5, black),
        ('BACKGROUND', (0, 0), (-1, 0), HexColor('#E0E0E0')),
    ]))
    
    story.append(summary_table)
    story.append(Spacer(1, 10))
    
    # Add totals summary in the same style as Connection Sessions
    total_flight_time = sum(drone['statistics']['flight_duration'] for drone in operation_data['drones'])
    total_distance = sum(drone['statistics']['total_distance'] for drone in operation_data['drones'])
    total_uavs = len(operation_data['drones'])
    
    # Calculate detection session totals
    total_detection_sessions = sum(len(drone['detection_sessions']) for drone in operation_data['drones'])
    total_detection_time = sum(
        sum(s['duration_minutes'] for s in drone['detection_sessions']) 
        for drone in operation_data['drones']
    )
    
    # Calculate operation-wide unique detections (each track_id counted only once across all sessions)
    all_track_ids = set()
    for drone in operation_data['drones']:
        for detection_session in drone['detection_sessions']:
            try:
                session_id = detection_session.get('session_id', None)
                if session_id:
                    # Get unique track_ids for this session and add to the global set
                    session_track_ids = DetectedObject.objects.filter(
                        detection_session_id=session_id,
                        track_id__isnull=False
                    ).values_list('track_id', flat=True).distinct()
                    all_track_ids.update(session_track_ids)
            except Exception as e:
                print(f"DEBUG: Error getting track_ids for session: {e}")
                continue
    
    total_unique_detections = len(all_track_ids)
    
    totals_text = f"Total UAVs: {total_uavs} | Total Flight Time: {format_flight_duration(total_flight_time)} | Total Distance: {format_distance(total_distance)}"
    if total_detection_sessions > 0:
        totals_text += f" | Detection Sessions: {total_detection_sessions} | Detection Time: {format_flight_duration(total_detection_time)} | Unique Objects (operation-wide): {total_unique_detections}"
    
    story.append(Paragraph(totals_text, caption_style))
    story.append(Spacer(1, 20))
    
    # Add missions table
    create_missions_table(story, operation_data)


def create_missions_table(story, operation_data):
    """Create the missions table section"""
    story.append(Paragraph("Missions", heading_style))
    
    if not operation_data['missions']:
        story.append(Paragraph("No missions found for UAVs currently in this operation.", body_style))
        story.append(Spacer(1, 20))
        return
    
    # Add note about mission filtering
    # story.append(Paragraph("Note: Only showing missions for UAVs currently assigned to this operation.", caption_style))
    # story.append(Spacer(1, 8))
    
    # Create missions table
    missions_data = [['Time', 'UAV', 'Type', 'User']]
    
    for mission in operation_data['missions']:
        # Format time from mission table
        formatted_time = mission.time.strftime('%Y-%m-%d %H:%M:%S')
        
        # Get UAV from mission log with action="START_MISSION"
        try:
            mission_log = MissionLog.objects.filter(
                mission=mission, 
                action="START_MISSION"
            ).first()
            uav_name = mission_log.drone.drone_name if mission_log and mission_log.drone else 'N/A'
        except:
            uav_name = 'N/A'
        
        # Get mission type from mission table and format it
        mission_type = format_mission_type(mission.mission_type)
        
        # Get user from mission table
        user_name = str(mission.user) if mission.user else 'N/A'
        
        missions_data.append([
            formatted_time,
            uav_name,
            mission_type,
            user_name
        ])
    
    missions_table = Table(missions_data, colWidths=[2*inch, 1.5*inch, 1.5*inch, 1.5*inch])
    missions_table.setStyle(TableStyle([
        ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ('FONTNAME', (0, 1), (-1, -1), 'Helvetica'),
        ('FONTSIZE', (0, 0), (-1, -1), 9),
        ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('GRID', (0, 0), (-1, -1), 0.5, black),
        ('BACKGROUND', (0, 0), (-1, 0), HexColor('#E0E0E0')),
    ]))
    
    story.append(missions_table)
    story.append(Spacer(1, 10))
    
    # Add missions summary
    total_missions_count = len(operation_data['missions'])
    
    # Get unique UAVs from mission logs with START_MISSION action
    mission_logs_with_drones = []
    for mission in operation_data['missions']:
        try:
            mission_log = MissionLog.objects.filter(
                mission=mission, 
                action="START_MISSION"
            ).first()
            if mission_log and mission_log.drone:
                mission_logs_with_drones.append(mission_log.drone.drone_name)
        except:
            pass
    
    unique_uavs = len(set(mission_logs_with_drones))
    unique_users = len(set(str(mission.user) for mission in operation_data['missions'] if mission.user))
    
    missions_summary = f"Total Missions (current UAVs): {total_missions_count} | UAVs Involved: {unique_uavs} | Users: {unique_users}"
    
    story.append(Paragraph(missions_summary, caption_style))
    story.append(Spacer(1, 20))


def create_drone_details(story, drone_data, enhanced_maps=True):
    """Create detailed section for a specific drone"""
    story.append(PageBreak())
    story.append(Paragraph(f"UAV Details: {drone_data['name']}", heading_style))
    
    # Drone information
    info_data = [
        ['UAV Name:', drone_data['name']],
        ['Model:', drone_data['model'] or 'N/A'],
        ['Type:', drone_data['type'] or 'N/A'],
        ['Configuration:', drone_data['configuration'] or 'N/A'],
        # ['Connection Status:', 'Connected' if drone_data['is_connected'] else 'Disconnected'],
        
    ]
    
    info_table = Table(info_data, colWidths=[1.5*inch, 2*inch])
    info_table.setStyle(TableStyle([
        ('FONTNAME', (0, 0), (-1, -1), 'Helvetica'),
        ('FONTSIZE', (0, 0), (-1, -1), 10),
        ('FONTNAME', (0, 0), (0, -1), 'Helvetica-Bold'),
        ('ALIGN', (0, 0), (0, -1), 'RIGHT'),
        ('ALIGN', (1, 0), (1, -1), 'LEFT'),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('LEFTPADDING', (0, 0), (-1, -1), 6),
        ('RIGHTPADDING', (0, 0), (-1, -1), 6),
        ('TOPPADDING', (0, 0), (-1, -1), 3),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 3),
        ('GRID', (0, 0), (-1, -1), 0.5, black),
    ]))
    
    story.append(info_table)
    story.append(Spacer(1, 15))
    
    # Flight statistics
    story.append(Paragraph("Flight Statistics", subheading_style))
    
    stats = drone_data['statistics']
    stats_data = [
        ['Flight Duration:', format_flight_duration(stats['flight_duration'])],
        ['Total Distance:', format_distance(stats['total_distance'])],  # Use formatted distance
        ['Maximum Altitude:', f"{stats['max_altitude']} m"],
        ['Maximum Speed:', f"{stats['max_speed']} m/s"],
        ['Average Speed:', f"{stats['avg_speed']} m/s"],
        ['Missions:', str(drone_data['missions'])],
    ]
    
    stats_table = Table(stats_data, colWidths=[2*inch, 2*inch])
    stats_table.setStyle(TableStyle([
        ('FONTNAME', (0, 0), (-1, -1), 'Helvetica'),
        ('FONTSIZE', (0, 0), (-1, -1), 10),
        ('FONTNAME', (0, 0), (0, -1), 'Helvetica-Bold'),
        ('ALIGN', (0, 0), (0, -1), 'RIGHT'),
        ('ALIGN', (1, 0), (1, -1), 'LEFT'),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('LEFTPADDING', (0, 0), (-1, -1), 6),
        ('RIGHTPADDING', (0, 0), (-1, -1), 6),
        ('TOPPADDING', (0, 0), (-1, -1), 3),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 3),
        ('GRID', (0, 0), (-1, -1), 0.5, black),
    ]))
    
    story.append(stats_table)
    story.append(Spacer(1, 15))
    
    # LiveStream Sessions table
    if drone_data['sessions']:
        story.append(Paragraph("Connection Sessions", subheading_style))

        # Create sessions table
        sessions_data = [['Start Time', 'End Time', 'Duration', 'Distance']]
        
        for session in drone_data['sessions']:
            sessions_data.append([
                session['start_time'],
                session['end_time'],
                session['duration_formatted'],
                session['distance_formatted']
            ])
        
        sessions_table = Table(sessions_data, colWidths=[2.2*inch, 2.2*inch, 1*inch, 1*inch])
        
        # Base table style
        base_style = [
            ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
            ('FONTNAME', (0, 1), (-1, -1), 'Helvetica'),
            ('FONTSIZE', (0, 0), (-1, -1), 9),
            ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
            ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
            ('GRID', (0, 0), (-1, -1), 0.5, black),
            ('BACKGROUND', (0, 0), (-1, 0), HexColor('#E0E0E0')),
            ('LEFTPADDING', (0, 0), (-1, -1), 6),
            ('RIGHTPADDING', (0, 0), (-1, -1), 6),
            ('TOPPADDING', (0, 0), (-1, -1), 3),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 3),
        ]
        
        sessions_table.setStyle(TableStyle(base_style))
        
        story.append(sessions_table)
        story.append(Spacer(1, 10))
        
        # Summary information
        total_sessions = len(drone_data['sessions'])
        total_flight_time = sum(s['duration_minutes'] for s in drone_data['sessions'])
        total_session_distance = sum(s['distance_meters'] for s in drone_data['sessions'])
        active_sessions = len([s for s in drone_data['sessions'] if s['status'] == 'Active'])
        
        summary_text = f"Total Sessions: {total_sessions}"
        if active_sessions > 0:
            summary_text += f" ({active_sessions} active)"
        summary_text += f" | Total Flight Time: {format_flight_duration(total_flight_time)}"
        summary_text += f" | Total Distance: {format_distance(total_session_distance)}"
        
        story.append(Paragraph(summary_text, caption_style))
        story.append(Spacer(1, 15))
    # else:
    #     story.append(Paragraph("No connection sessions found for this UAV.", body_style))
    
    # Detection Sessions table
    if drone_data['detection_sessions']:
        story.append(Paragraph("Detection Sessions", subheading_style))

        # Create detection sessions table
        detection_sessions_data = [['Start Time', 'End Time', 'Duration', 'User', 'Status', 'Detections']]
        
        for session in drone_data['detection_sessions']:
            detection_sessions_data.append([
                session['start_time'],
                session['end_time'],
                session['duration_formatted'],
                session['user'],
                session['status'],
                str(session['unique_detections_count'])
            ])
        
        detection_sessions_table = Table(detection_sessions_data, colWidths=[1.5*inch, 1.5*inch, 1*inch, 1*inch, 0.8*inch, 0.8*inch])
        
        # Base table style
        detection_base_style = [
            ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
            ('FONTNAME', (0, 1), (-1, -1), 'Helvetica'),
            ('FONTSIZE', (0, 0), (-1, -1), 9),
            ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
            ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
            ('GRID', (0, 0), (-1, -1), 0.5, black),
            ('BACKGROUND', (0, 0), (-1, 0), HexColor('#E0E0E0')),
            ('LEFTPADDING', (0, 0), (-1, -1), 6),
            ('RIGHTPADDING', (0, 0), (-1, -1), 6),
            ('TOPPADDING', (0, 0), (-1, -1), 3),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 3),
        ]
        
        detection_sessions_table.setStyle(TableStyle(detection_base_style))
        
        story.append(detection_sessions_table)
        story.append(Spacer(1, 10))
        
        # Detection sessions summary information
        total_detection_sessions = len(drone_data['detection_sessions'])
        total_detection_time = sum(s['duration_minutes'] for s in drone_data['detection_sessions'])
        active_detection_sessions = len([s for s in drone_data['detection_sessions'] if s['status'] == 'Active'])
        total_unique_detections = sum(s['unique_detections_count'] for s in drone_data['detection_sessions'])
        
        detection_summary_text = f"Total Detection Sessions: {total_detection_sessions}"
        if active_detection_sessions > 0:
            detection_summary_text += f" ({active_detection_sessions} active)"
        detection_summary_text += f" | Total Detection Time: {format_flight_duration(total_detection_time)}"
        detection_summary_text += f" | Total Detections (all sessions): {total_unique_detections}"
        
        story.append(Paragraph(detection_summary_text, caption_style))
        story.append(Spacer(1, 15))
    
    
    # Flight path map
    if drone_data['telemetry_points']:
        # story.append(Paragraph("Individual Flight Path Map", subheading_style))
        
        # Create the map
        map_flowable = FlightPathMapFlowable(drone_data, width=6.3*inch, height=4.5*inch, enhanced_maps=enhanced_maps)
        story.append(map_flowable)
        story.append(Spacer(1, 0))
        story.append(Paragraph(f"Flight path visualization for {drone_data['name']}", caption_style))
    else:
        story.append(Paragraph("No flight data available for path visualization.", body_style))


def create_operation_pdf(operation_name, output_path, start_time=None, end_time=None, enhanced_maps=True):
    """Main function to create the operation report PDF"""
    
    # Gather data
    operation_data = get_operation_data(operation_name, start_time, end_time)
    
    # Create PDF document
    doc = BaseDocTemplate(
        output_path,
        pagesize=A4,
        title=f"Operation Report - {operation_name}",
        author="AIDERS Platform"
    )
    
    # Define frame with margins for header and footer
    # Increased top margin on first page for logo + subtitle, increased bottom margin for footer
    frame_first_page = Frame(
        x1=inch,
        y1=1.2*inch,  # Bottom margin for footer
        width=page_width - 2*inch,
        height=page_height - 3.7*inch,  # Adjusted for header with subtitle and footer
        leftPadding=0,
        bottomPadding=0,
        rightPadding=0,
        topPadding=0,
        showBoundary=0
    )
    
    frame_other_pages = Frame(
        x1=inch,
        y1=1.2*inch,  # Bottom margin for footer
        width=page_width - 2*inch,
        height=page_height - 2.2*inch,  # Adjusted for footer only
        leftPadding=0,
        bottomPadding=0,
        rightPadding=0,
        topPadding=0,
        showBoundary=0
    )
    
    # Create custom page templates with header and footer
    # Generate shared timestamp for consistent report IDs across all pages
    shared_generation_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S UTC')
    
    first_page_template = HeaderFooterPageTemplate(
        id='first_page',
        frames=[frame_first_page],
        pagesize=A4,
        operation_name=operation_name,
        generation_time=shared_generation_time
    )
    
    normal_template = HeaderFooterPageTemplate(
        id='normal',
        frames=[frame_other_pages],
        pagesize=A4,
        operation_name=operation_name,
        generation_time=shared_generation_time
    )
    
    doc.addPageTemplates([first_page_template, normal_template])
    
    # Build story
    story = []
    
    # Start with first page template
    story.append(Spacer(1, 0))  # Add space after header logo
    
    # Create sections
    create_operation_summary(story, operation_data, enhanced_maps)
    
    # Switch to normal template for subsequent pages
    story.append(NextPageTemplate('normal'))
    # story.append(PageBreak())
    
    # create_drones_summary(story, operation_data)
    
    # Create detailed sections for each drone
    for drone_data in operation_data['drones']:
        create_drone_details(story, drone_data, enhanced_maps)
    
    # Build PDF
    doc.build(story)
    
    return output_path


def generate_operation_report(operation_name, start_time=None, end_time=None, enhanced_maps=True):
    """
    Generate operation report and return the file path
    
    Args:
        operation_name (str): Name of the operation
        start_time (datetime, optional): Start time for telemetry data
        end_time (datetime, optional): End time for telemetry data
        enhanced_maps (bool, optional): Use real map tiles instead of synthetic background
    
    Returns:
        str: Path to generated PDF file
    """
    
    # Ensure reports directory exists
    reports_dir = 'operation_reports'
    if not os.path.exists(default_storage.path(reports_dir)):
        os.makedirs(default_storage.path(reports_dir))
    
    # Generate filename
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f"operation_report_{operation_name}_{timestamp}.pdf"
    file_path = os.path.join(reports_dir, filename)
    full_path = default_storage.path(file_path)
    
    # Create the PDF
    create_operation_pdf(operation_name, full_path, start_time, end_time, enhanced_maps)
    
    return file_path


def parse_report_date(value, end_of_day=False):
    """
    Parse a date string coming from the report form into a datetime

    Accepts both 'YYYY-MM-DDTHH:MM' (datetime-local input) and 'YYYY-MM-DD'.
    Date-only values are expanded to the start of the day, or to the end of the
    day when end_of_day is True.
    """
    for date_format in ('%Y-%m-%dT%H:%M:%S', '%Y-%m-%dT%H:%M', '%Y-%m-%d %H:%M:%S', '%Y-%m-%d %H:%M'):
        try:
            return datetime.strptime(value, date_format)
        except ValueError:
            continue

    parsed = datetime.strptime(value, '%Y-%m-%d')
    if end_of_day:
        parsed = parsed.replace(hour=23, minute=59, second=59)
    return parsed


class OperationReportGenerator:
    """
    Main class for generating comprehensive operation reports with flight paths and statistics
    """
    
    def __init__(self):
        """Initialize the report generator"""
        self.report_data = None
    
    def generate_operation_report(self, operation, start_date=None, end_date=None, 
                                 include_flight_paths=True, include_statistics=True, enhanced_maps=True):
        """
        Generate a comprehensive operation report PDF
        
        Args:
            operation: Django Operation model instance
            start_date: Start date for filtering telemetry data (overrides operation start if provided)
            end_date: End date for filtering telemetry data (overrides operation end if provided)
            include_flight_paths: Whether to include flight path maps
            include_statistics: Whether to include detailed statistics
            enhanced_maps: Whether to use enhanced map visualization
            
        Returns:
            BytesIO: PDF file buffer
        """
        
        # Convert dates to datetime if needed, or use operation's own dates
        start_time = None
        end_time = None
        
        if start_date:
            if isinstance(start_date, str):
                start_time = parse_report_date(start_date)
            else:
                start_time = start_date
        # If no start_date provided, use operation's start date (this will be handled by get_operation_data)

        if end_date:
            if isinstance(end_date, str):
                end_time = parse_report_date(end_date, end_of_day=True)
            else:
                end_time = end_date
        # If no end_date provided, use operation's end date if it exists (this will be handled by get_operation_data)

        # Create temporary file path
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        temp_filename = f"temp_operation_report_{operation.operation_name}_{timestamp}.pdf"
        
        # Ensure reports directory exists
        reports_dir = 'operation_reports'
        try:
            if not os.path.exists(default_storage.path(reports_dir)):
                os.makedirs(default_storage.path(reports_dir))
        except:
            # Fallback to temp directory
            reports_dir = '/tmp'
        
        temp_path = os.path.join(reports_dir, temp_filename)
        full_temp_path = default_storage.path(temp_path) if reports_dir != '/tmp' else temp_path
        
        try:
            # Generate the PDF using existing function
            create_operation_pdf(
                operation_name=operation.operation_name,
                output_path=full_temp_path,
                start_time=start_time,
                end_time=end_time,
                enhanced_maps=enhanced_maps
            )
            
            # Read the PDF into a BytesIO buffer
            pdf_buffer = BytesIO()
            with open(full_temp_path, 'rb') as pdf_file:
                pdf_buffer.write(pdf_file.read())
            
            pdf_buffer.seek(0)
            
            # Clean up temporary file
            try:
                os.remove(full_temp_path)
            except:
                pass
            
            return pdf_buffer
            
        except Exception as e:
            # Clean up temporary file in case of error
            try:
                if os.path.exists(full_temp_path):
                    os.remove(full_temp_path)
            except:
                pass
            raise e
    
    def get_operation_summary(self, operation_name):
        """
        Get a summary of operation data without generating full report
        
        Args:
            operation_name: Name of the operation
            
        Returns:
            dict: Operation summary data
        """
        try:
            operation_data = get_operation_data(operation_name)
            
            summary = {
                'operation_name': operation_data['operation'].operation_name,
                'total_drones': operation_data['total_drones'],
                'active_drones': operation_data['active_drones'],
                'total_missions': operation_data['total_missions'],
                'total_detection_sessions': sum(len(drone['detection_sessions']) for drone in operation_data['drones']),
                'created_date': operation_data['operation'].created_at,
                'ended_date': operation_data['operation'].ended_at,
                'drones_summary': []
            }
            
            for drone_data in operation_data['drones']:
                drone_summary = {
                    'name': drone_data['name'],
                    'model': drone_data['model'],
                    'is_connected': drone_data['is_connected'],
                    'missions': drone_data['missions'],
                    'flight_duration': drone_data['statistics']['flight_duration'],
                    'total_distance': drone_data['statistics']['total_distance'],
                    'telemetry_points_count': len(drone_data['telemetry_points']),
                    'detection_sessions_count': len(drone_data['detection_sessions']),
                    'total_detection_time': sum(s['duration_minutes'] for s in drone_data['detection_sessions'])
                }
                summary['drones_summary'].append(drone_summary)
            
            return summary
            
        except Exception as e:
            raise ValueError(f"Error getting operation summary: {str(e)}")


# Custom Page Template classes for header and footer
class HeaderFooterPageTemplate(PageTemplate):
    """Custom page template with header and footer"""
    
    def __init__(self, id, frames, pagesize=A4, operation_name=None, generation_time=None, **kwargs):
        PageTemplate.__init__(self, id, frames, pagesize=pagesize, **kwargs)
        self.pagesize = pagesize
        self.operation_name = operation_name
        self.generation_time = generation_time or datetime.now().strftime('%Y-%m-%d %H:%M:%S UTC')
        
        # Generate unique report ID hash (use operation_name for consistency, not template id)
        import hashlib
        hash_input = f"{operation_name or 'unknown'}_{self.generation_time}"
        self.report_hash = hashlib.md5(hash_input.encode()).hexdigest()[:8].upper()
        self.report_id = f"RPT-{self.report_hash}"
    
    def beforeDrawPage(self, canvas, doc):
        """Add header and footer to each page"""
        canvas.saveState()
        
        # Add footer with KIOS logo and copyright
        self._add_footer(canvas, doc)
        
        # Add header with AIDERS logo and operation details on first page
        if doc.page == 1:
            self._add_header(canvas, doc)
        
        # Add vertical report ID in left margin on all pages
        self._add_report_id(canvas, doc)
        
        canvas.restoreState()
    
    def _draw_centered_text(self, canvas, x, y, text, font_name, font_size):
        """Helper method to draw centered text"""
        canvas.setFont(font_name, font_size)
        text_width = canvas.stringWidth(text, font_name, font_size)
        canvas.drawString(x - text_width / 2, y, text)
    
    def _add_report_id(self, canvas, doc):
        """Add vertical report ID in left margin"""
        try:
            # Save current state
            canvas.saveState()
            
            # Position in left margin, vertically centered
            x = 25  # 25 points from left edge
            y = self.pagesize[1] / 2  # Center vertically
            
            # Set font and color
            canvas.setFont("Helvetica", 8)
            canvas.setFillColorRGB(0.5, 0.5, 0.5)  # Light gray
            
            # Rotate text 90 degrees for vertical display
            canvas.rotate(90)
            
            # Draw the report ID (coordinates are rotated, so x and y are swapped)
            canvas.drawString(y - len(self.report_id) * 3, -x, self.report_id)
            
            # Restore state
            canvas.restoreState()
            
        except Exception as e:
            print(f"Error adding report ID: {str(e)}", flush=True)
            # Don't let this break the report generation
            pass
    
    def _add_header(self, canvas, doc):
        """Add header with AIDERS logo and operation details on first page"""
        try:
            # Get AIDERS logo path
            logo_path = os.path.join(settings.BASE_DIR, 'aiders', 'static', 'aiders', 'imgs', 'aiders_logo_with_text.png')
            # logo_path = os.path.join(settings.BASE_DIR, 'aiders', 'static', 'aiders', 'imgs', 'reaction.png')

            
            # Get platform version from environment
            platform_version = os.environ.get("VERSION", "")
            
            if os.path.exists(logo_path):
                # Position logo in header (centered)
                logo_width = 3 * inch
                logo_height = 1 * inch
                x = (self.pagesize[0] - logo_width) / 2
                y = self.pagesize[1] - 1.5 * inch
                
                canvas.drawImage(logo_path, x, y, width=logo_width, height=logo_height, preserveAspectRatio=True)
                
                # Add platform version in bottom right corner of logo area (very small text)
                if platform_version:
                    version_x = x + logo_width + 0  # points from right edge of logo
                    version_y = y + 15  # points from bottom edge of logo
                    canvas.setFont("Helvetica", 8)  # Very small font
                    canvas.setFillColorRGB(0.2, 0.2, 0.2)  # Gray color
                    canvas.drawRightString(version_x, version_y, f"v{platform_version}")
                    canvas.setFillColorRGB(0, 0, 0)  # Reset to black
                
                # Add operation subtitle below logo
                subtitle_y = y - 0.3 * inch
                if self.operation_name:
                    self._draw_centered_text(canvas, self.pagesize[0] / 2, subtitle_y, 
                                           f"Operation Report for: {self.operation_name}", 
                                           "Helvetica-Bold", 14)
                    
                    # Add generation datetime below subtitle
                    self._draw_centered_text(canvas, self.pagesize[0] / 2, subtitle_y - 0.2 * inch,
                                           f"Generated at: {self.generation_time}",
                                           "Helvetica", 10)
                
            else:
                # Fallback text if logo not found
                self._draw_centered_text(canvas, self.pagesize[0] / 2, self.pagesize[1] - 1 * inch,
                                       "AIDERS Platform", "Helvetica-Bold", 16)
                
                # Add platform version for fallback text too
                if platform_version:
                    canvas.setFont("Helvetica", 6)
                    canvas.setFillColorRGB(0.5, 0.5, 0.5)
                    canvas.drawRightString(self.pagesize[0] / 2 + 100, self.pagesize[1] - 1.2 * inch, f"v{platform_version}")
                    canvas.setFillColorRGB(0, 0, 0)
                
                # Add operation subtitle
                if self.operation_name:
                    self._draw_centered_text(canvas, self.pagesize[0] / 2, self.pagesize[1] - 1.3 * inch,
                                           f"Operation Report for: {self.operation_name}",
                                           "Helvetica-Bold", 14)
                    
                    # Add generation datetime
                    self._draw_centered_text(canvas, self.pagesize[0] / 2, self.pagesize[1] - 1.5 * inch,
                                           f"Generated at: {self.generation_time}",
                                           "Helvetica", 10)
                
        except Exception as e:
            print(f"Error adding header: {str(e)}", flush=True)
            # Get platform version for fallback too
            platform_version = os.environ.get("VERSION", "")
            
            # Fallback to text header if image fails
            self._draw_centered_text(canvas, self.pagesize[0] / 2, self.pagesize[1] - 1 * inch,
                                   "AIDERS Platform", "Helvetica-Bold", 16)
            
            # Add platform version for exception fallback
            if platform_version:
                canvas.setFont("Helvetica", 6)
                canvas.setFillColorRGB(0.5, 0.5, 0.5)
                canvas.drawRightString(self.pagesize[0] / 2 + 100, self.pagesize[1] - 1.2 * inch, f"v{platform_version}")
                canvas.setFillColorRGB(0, 0, 0)
            
            # Add operation subtitle even in fallback
            if self.operation_name:
                self._draw_centered_text(canvas, self.pagesize[0] / 2, self.pagesize[1] - 1.3 * inch,
                                       f"Operation Report for: {self.operation_name}",
                                       "Helvetica-Bold", 14)
                
                # Add generation datetime
                self._draw_centered_text(canvas, self.pagesize[0] / 2, self.pagesize[1] - 1.5 * inch,
                                       f"Generated at: {self.generation_time}",
                                       "Helvetica", 10)
    
    def _add_footer(self, canvas, doc):
        """Add footer with KIOS logo and copyright"""
        try:
            # Get KIOS logo path
            kios_logo_path = os.path.join(settings.BASE_DIR, 'aiders', 'static', 'aiders', 'imgs', 'kios_logo_white.jpg')
            # kios_logo_path = os.path.join(settings.BASE_DIR, 'aiders', 'static', 'aiders', 'imgs', 'reaction-logo.jpg')
            
            # Footer elements
            footer_y = 0.5 * inch
            
            if os.path.exists(kios_logo_path):
                # Position KIOS logo on the left
                logo_width = 0.8 * inch
                logo_height = 0.6 * inch
                canvas.drawImage(kios_logo_path, 1 * inch, footer_y - 0.22 * inch, 
                               width=logo_width, height=logo_height, preserveAspectRatio=True)
                
                # Copyright text next to logo
                canvas.setFont("Helvetica", 8)
                current_year = datetime.now().year
                copyright_text = f"© {current_year} KIOS Research and Innovation Center of Excellence"
                canvas.drawString(1 * inch + logo_width + 0.2 * inch, footer_y + 0.0 * inch, copyright_text)
            else:
                # Fallback text footer if logo not found
                canvas.setFont("Helvetica", 9)
                current_year = datetime.now().year
                copyright_text = f"© {current_year} KIOS Research and Innovation Center of Excellence"
                canvas.drawString(1 * inch, footer_y + 0.2 * inch, copyright_text)
            
            # Page number (simple format for now)
            canvas.setFont("Helvetica", 9)
            current_page = doc.page
            page_text = f"Page {current_page}"
            canvas.drawRightString(self.pagesize[0] - 1 * inch, footer_y + 0.03 * inch, page_text)
            
        except Exception as e:
            # Fallback footer
            canvas.setFont("Helvetica", 9)
            current_year = datetime.now().year
            copyright_text = f"© {current_year} KIOS Research and Innovation Center of Excellence"
            canvas.drawString(1 * inch, footer_y + 0.2 * inch, copyright_text)
            
            # Page number in fallback (simple format)
            current_page = doc.page
            page_text = f"Page {current_page}"
            canvas.drawRightString(self.pagesize[0] - 1 * inch, footer_y + 0.2 * inch, page_text)
            

# Legacy function wrapper for backward compatibility
def generate_operation_report_legacy(operation_name, start_time=None, end_time=None, enhanced_maps=True):
    """
    Legacy wrapper function for backward compatibility
    """
    return generate_operation_report(operation_name, start_time, end_time, enhanced_maps)


def calculate_actual_flight_duration(drone, start_time=None, end_time=None):
    """
    Calculate actual flight duration based on LiveStreamSession data
    This accounts for disconnection periods and only counts active flight time
    
    Args:
        drone: Drone model instance
        start_time: Start time filter (required for operation time range)
        end_time: End time filter (optional, only if operation has ended)
    
    Returns:
        float: Total flight duration in minutes
    """
    # Get all live stream sessions for this drone within the specified time range
    sessions_filter = {'drone': drone}
    if start_time:
        sessions_filter['start_time__gte'] = start_time
    if end_time:
        sessions_filter['start_time__lte'] = end_time
    
    sessions = LiveStreamSession.objects.filter(**sessions_filter).order_by('start_time')
    
    total_duration_seconds = 0
    
    for session in sessions:
        # Calculate duration for each session
        session_start = session.start_time
        session_end = session.end_time
        
        # If session is not ended, use current time or end_time filter
        if not session_end:
            if end_time:
                session_end = end_time
            else:
                session_end = datetime.now()
                # If timezone aware, use timezone aware current time
                if session_start.tzinfo:
                    session_end = timezone.now()
        
        # Apply time range filters
        if start_time and session_start < start_time:
            session_start = start_time
        if end_time and session_end > end_time:
            session_end = end_time
        
        # Only count positive durations
        if session_end > session_start:
            duration_seconds = (session_end - session_start).total_seconds()
            total_duration_seconds += duration_seconds
    
    # Convert to minutes
    return total_duration_seconds / 60


def format_flight_duration(duration_minutes):
    """
    Format flight duration from minutes to MM:SS format
    
    Args:
        duration_minutes: Duration in minutes
    
    Returns:
        str: Formatted duration string in MM:SS format
    """
    if duration_minutes == 0:
        return "00:00"
    
    # Convert minutes to total seconds
    total_seconds = int(duration_minutes * 60)
    
    # Calculate minutes and seconds
    minutes = total_seconds // 60
    seconds = total_seconds % 60
    
    return f"{minutes:02d}:{seconds:02d}"


def get_drone_livestream_sessions(drone, start_time=None, end_time=None):
    """
    Get LiveStreamSession data for a drone within specified time range
    
    Args:
        drone: Drone model instance
        start_time: Start time filter (required for operation time range)
        end_time: End time filter (optional, only if operation has ended)
    
    Returns:
        list: List of session dictionaries with formatted data
    """
    # Get all live stream sessions for this drone within the specified time range
    sessions_filter = {'drone': drone}
    if start_time:
        sessions_filter['start_time__gte'] = start_time
    if end_time:
        sessions_filter['start_time__lte'] = end_time
    
    sessions = LiveStreamSession.objects.filter(**sessions_filter).order_by('start_time')
    
    print(f"DEBUG: Found {len(sessions)} sessions for drone {drone.drone_name}")
    
    session_data = []
    for session in sessions:
        session_start = session.start_time
        session_end = session.end_time
        
        print(f"DEBUG: Processing session for {drone.drone_name}: {session_start} to {session_end}")
        
        # Calculate duration
        if session_end:
            duration_seconds = (session_end - session_start).total_seconds()
            duration_minutes = duration_seconds / 60
            status = "Completed"
            actual_session_end = session_end
        else:
            # Active session - calculate duration up to now or end_time
            current_time = end_time if end_time else timezone.now()
            duration_seconds = (current_time - session_start).total_seconds()
            duration_minutes = duration_seconds / 60
            status = "Active" if session.is_active else "Incomplete"
            actual_session_end = current_time
        
        # Apply time range filters
        if start_time and session_start < start_time:
            session_start = start_time
        if end_time and actual_session_end > end_time:
            actual_session_end = end_time
        
        # Only count positive durations
        if actual_session_end > session_start:
            distance_covered = calculate_session_distance(drone, session_start, actual_session_end)
            
            session_info = {
                'start_time': session_start.strftime('%Y-%m-%d %H:%M:%S UTC'),
                'end_time': session_end.strftime('%Y-%m-%d %H:%M:%S UTC') if session_end else 'N/A',
                'duration_minutes': round(duration_minutes, 2),
                'duration_formatted': format_flight_duration(duration_minutes),
                'distance_meters': round(distance_covered, 2),
                'distance_formatted': format_distance(distance_covered),
                'status': status,
                'is_active': session.is_active
            }
            
            session_data.append(session_info)
    
    print(f"DEBUG: Processed {len(session_data)} valid sessions for drone {drone.drone_name}")
    return session_data


def get_drone_detection_sessions(drone, start_time=None, end_time=None):
    """
    Get DetectionSession data for a drone within specified time range
    
    Args:
        drone: Drone model instance
        start_time: Start time filter (required for operation time range)
        end_time: End time filter (optional, only if operation has ended)
    
       
    Returns:
        list: List of detection session dictionaries with formatted data
    """
    # Get all detection sessions for this drone within the specified time range
    sessions_filter = {'drone': drone}
    if start_time:
        sessions_filter['start_time__gte'] = start_time
    if end_time:
        sessions_filter['start_time__lte'] = end_time
    
    sessions = DetectionSession.objects.filter(**sessions_filter).order_by('start_time')
    
    print(f"DEBUG: Found {len(sessions)} detection sessions for drone {drone.drone_name}")
    
    session_data = []
    for session in sessions:
        session_start = session.start_time
        session_end = session.end_time
        
        print(f"DEBUG: Processing detection session for {drone.drone_name}: {session_start} to {session_end}")
        
        # Calculate duration
        if session_end:
            duration_seconds = (session_end - session_start).total_seconds()
            duration_minutes = duration_seconds / 60
            status = "Completed"
            actual_session_end = session_end
        else:
            # Active session - calculate duration up to now or end_time
            current_time = end_time if end_time else timezone.now()
            duration_seconds = (current_time - session_start).total_seconds()
            duration_minutes = duration_seconds / 60
            status = "Active" if session.is_active else "Incomplete"
            actual_session_end = current_time
        
        # Apply time range filters
        if start_time and session_start < start_time:
            session_start = start_time
        if end_time and actual_session_end > end_time:
            actual_session_end = end_time
        
        # Only count positive durations
        if actual_session_end > session_start:
            duration_seconds = (actual_session_end - session_start).total_seconds()
            duration_minutes = duration_seconds / 60
            
            # Count unique detections (by track_id) for this session
            # Only count the latest detection for each unique track_id
            unique_detections_count = DetectedObject.objects.filter(
                detection_session=session,
                track_id__isnull=False
            ).values('track_id').distinct().count()
            
            session_info = {
                'session_id': session.id,  # Add session ID for heatmap
                'start_time': session_start.strftime('%Y-%m-%d %H:%M:%S UTC'),
                'end_time': session_end.strftime('%Y-%m-%d %H:%M:%S UTC') if session_end else 'N/A',
                'duration_minutes': round(duration_minutes, 2),
                'duration_formatted': format_flight_duration(duration_minutes),
                'status': status,
                'is_active': session.is_active,
                'user': str(session.user),
                'latest_frame_url': session.latest_frame_url or 'N/A',
                'unique_detections_count': unique_detections_count
            }
            
            session_data.append(session_info)
    
    print(f"DEBUG: Processed {len(session_data)} valid detection sessions for drone {drone.drone_name}")
    return session_data


def calculate_session_distance(drone, session_start, session_end):
    """
    Calculate distance covered by a drone during a specific session
    
    Args:
        drone: Drone model instance
        session_start: Session start datetime
        session_end: Session end datetime
    
    Returns:
        float: Distance covered in meters during the session
    """
    # Get telemetry data for this specific session
    telemetry_data = Telemetry.objects.filter(
        drone=drone,
        time__gte=session_start,
        time__lte=session_end
    ).order_by('time')
    
    print(f"DEBUG: Session distance calculation for {drone.drone_name}: {len(telemetry_data)} telemetry points between {session_start} and {session_end}")
    
    if len(telemetry_data) < 2:
        print(f"DEBUG: Not enough telemetry data for {drone.drone_name}: {len(telemetry_data)} points")
        return 0.0
    
    total_distance = 0.0
    prev_point = None
    valid_points_count = 0
    
    for telemetry in telemetry_data:
        # Filter out invalid GPS coordinates (0,0 before GPS lock)
        if not is_valid_gps_coordinate(telemetry.lat, telemetry.lon):
            continue
            
        valid_points_count += 1
        
        if prev_point is not None:
            # Calculate distance between consecutive points
            lat_diff = telemetry.lat - prev_point.lat
            lon_diff = telemetry.lon - prev_point.lon
            # Simple distance calculation (not accounting for Earth's curvature)
            distance = math.sqrt(lat_diff**2 + lon_diff**2) * 111000  # Rough conversion to meters
            total_distance += distance
        
        prev_point = telemetry
    
    print(f"DEBUG: Session distance for {drone.drone_name}: {valid_points_count} valid points, {total_distance:.2f}m total distance")
    return total_distance


def format_distance(distance_meters):
    """
    Format distance from meters to human readable format
    
    Args:
        distance_meters: Distance in meters
    
    Returns:
        str: Formatted distance string
    """
    if distance_meters < 1000:
        return f"{distance_meters:.1f} m"
    else:
        return f"{distance_meters/1000:.2f} km"


def format_mission_type(mission_type):
    """
    Format mission type for display with more readable names
    
    Args:
        mission_type: Raw mission type from database
    
    Returns:
        str: Formatted mission type for display
    """
    if not mission_type:
        return 'N/A'
    
    # Mission type replacements
    type_mappings = {
        'SEARCH_AND_RESCUE_MISSION': 'GRID',
        'NORMAL_MISSION': 'POINT TO POINT'
    }
    
    return type_mappings.get(mission_type, mission_type)


def is_valid_gps_coordinate(lat, lon):
    """Check if GPS coordinates are valid (not 0,0 and within reasonable bounds)"""
    if lat is None or lon is None:
        return False
    if lat == 0.0 and lon == 0.0:
        return False
    if abs(lat) > 90 or abs(lon) > 180:
        return False
    return True


def filter_valid_telemetry_points(telemetry_points):
    """
    Filter telemetry points to only include those with valid GPS coordinates
    
    Args:
        telemetry_points: List of telemetry data points
    
    Returns:
        list: Filtered list with only valid GPS coordinates
    """
    return [point for point in telemetry_points if is_valid_gps_coordinate(point['lat'], point['lon'])]


def segment_flight_paths(telemetry_points, max_time_gap_seconds=30, max_distance_gap_meters=500):
    """
    Segment telemetry points into continuous flight sessions based on time and distance gaps.
    
    This function solves the issue where disconnected flight sessions (e.g., drone connects at 
    location A, flies around, disconnects, then connects later at location B) were being 
    connected with straight lines in flight path visualizations. Now they are properly 
    separated into distinct flight segments.
    
    Args:
        telemetry_points: List of telemetry data points (must have 'time', 'lat', 'lon' keys)
        max_time_gap_seconds: Maximum time gap between consecutive points to consider them connected (default: 30 seconds)
        max_distance_gap_meters: Maximum distance gap between consecutive points to consider them connected (default: 500 meters)
    
    Returns:
        list: List of flight segments, where each segment is a list of telemetry points
    """
    if not telemetry_points or len(telemetry_points) < 2:
        return [telemetry_points] if telemetry_points else []
    
    # First filter valid points
    valid_points = filter_valid_telemetry_points(telemetry_points)
    if len(valid_points) < 2:
        return [valid_points] if valid_points else []
    
    # Sort by time to ensure correct order
    sorted_points = sorted(valid_points, key=lambda x: x['time'])
    
    segments = []
    current_segment = [sorted_points[0]]
    
    for i in range(1, len(sorted_points)):
        current_point = sorted_points[i]
        previous_point = sorted_points[i-1]
        
        # Calculate time gap
        time_gap = (current_point['time'] - previous_point['time']).total_seconds()
        
        # Calculate distance gap using Haversine formula (approximate)
        lat_diff = current_point['lat'] - previous_point['lat']
        lon_diff = current_point['lon'] - previous_point['lon']
        
        # Convert to meters (rough approximation: 1 degree ≈ 111km)
        distance_gap = math.sqrt(lat_diff**2 + lon_diff**2) * 111000
        
        # Check if we should start a new segment
        if time_gap > max_time_gap_seconds or distance_gap > max_distance_gap_meters:
            # End current segment and start a new one
            if len(current_segment) >= 2:  # Only add segments with at least 2 points
                segments.append(current_segment)
            current_segment = [current_point]
        else:
            # Continue current segment
            current_segment.append(current_point)
    
    # Add the last segment
    if len(current_segment) >= 2:  # Only add segments with at least 2 points
        segments.append(current_segment)
    elif len(current_segment) == 1 and len(segments) == 0:
        # Special case: if we only have one point and no segments, still include it
        segments.append(current_segment)
    
    print(f"DEBUG: Segmented {len(valid_points)} telemetry points into {len(segments)} flight segments")
    return segments


# Custom styles
