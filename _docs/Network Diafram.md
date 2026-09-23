# Network Architecture Overview

This document provides an overview of the network architecture depicted in the diagram. The aim is to help new developers understand how the components are interconnected and how various services communicate within the system.

## Diagram Legend

*   Orange Arrows:
    *   Purpose: Routes incoming requests to the correct container via NGINX.
    *   Mechanism: NGINX uses different paths like `/name_container/` to direct the requests to the appropriate container.

*   Red Arrows:
    *   Purpose: Handles database connections.
    *   Mechanism: Connections go through HAProxy using the main machine's IP and the relevant port for load balancing and routing.

*   Brown Arrows:
    *   Purpose: Transmits data to the platform for persistent storage.
    *   Mechanism: Data collected from various services is saved to the platform using these paths.

*   Blue Arrows:
    *   Purpose: Connects containers to the database within the Docker network.
    *   Mechanism: Ensures that services have internal access to the databases without needing external routes.

*   Purple Arrows:
    *   Purpose: Facilitates the connection between containers and the RTMP server for live streaming.
    *   Mechanism: RTMP streams are fed from containers to the streaming service.

*   Green Arrows:
    *   Purpose: Connects the NGINX instance from the main PC to the remote server.
    *   Mechanism: This allows CV and LSC containers to run on a separate machine while still being accessible via NGINX.

## System Components

*   Main PC: Acts as the primary control node, managing traffic and orchestration.
*   User PC: Provides the interface for end-users to interact with the system.
*   Remote Network: Contains additional services like CV and LSC that are hosted on separate infrastructure for load distribution.
*   NGINX: Manages routing for various services, including remote and local containers.

## Key Containers and Services

*   ROS (ros): Handles robotic operating system tasks, managing communication between hardware and software layers.

*   MAV (mavlink): Manages MAVLink communication, a lightweight messaging protocol for drones and similar devices.

*   RTMP (Real-Time Messaging Protocol): Handles real-time media streaming services, providing an endpoint for live video feeds.

*   NGINX: The main reverse proxy that routes requests to different services within the network based on path rules.

*   WSI (websocket in): Manages WebSocket connections coming into the system, enabling real-time communication with the front end.

*   WS (websocket): General-purpose WebSocket service, enabling bi-directional communication between clients and servers.

*   Web (web): A Django-based web application that serves as the primary back end and front end for user interaction.

*   ODM (open drone map): Provides drone data processing capabilities, enabling aerial imagery to be processed into useful map data.

*   GEO (offline map server): Hosts offline maps for location services, providing geographic data to the platform.

*   LSC (live stream capture): Captures and handles live streams, ensuring they are processed and made available for viewing.

*   CV (computer vision): Handles computer vision tasks such as object detection and tracking, leveraging AI models to analyze video feeds.

*   ALG (algorithms): Processes various algorithms for decision-making, analytics, and optimizations based on real-time and historical data.

*   CCD (check client disconnect): Monitors the status of client connections, ensuring that disconnections are detected and handled gracefully.

*   MySQL: The primary database for storing application and user data, providing persistence across the platform.

*   HAProxy: Used for load balancing and distributing requests across multiple database instances, ensuring high availability and reliability.

*   NGINXR: A remote NGINX instance used to route requests between remote services and the main network.

*   LSCR (live stream capture remote): A remote instance of the live stream capture service, handling streams on a separate machine.

*   CVR (computer vision remote): A remote instance of the computer vision service, enabling distributed processing on a different machine.

## Detailed Data Flow

*   Client Requests:
    *   Requests are routed to NGINX, which then determines the correct service based on path rules (orange arrows).
*   Database Access:
    *   Service containers connect to MySQL either directly (blue arrows) or through HAProxy (red arrows) for load balancing.
*   Platform Data Submission:
    *   Data flows from processing containers to the platform using the brown arrows, ensuring persistent storage.
*   Live Streaming:
    *   Media streams are handled via RTMP connections (purple arrows) and routed through NGINX for external access.
*   Remote Service Communication:
    *   NGINX in the main PC routes requests to the remote network (green arrows) to access containers like CV and LSC, enabling distributed computation.

## Additional Notes for Developers

*   Networking: Ensure that Docker networks are configured correctly to allow inter-container communication based on the architecture shown.
*   Ports and IPs: Be mindful of port conflicts when deploying on the same host. Use the correct IP addresses for HAProxy and NGINX routing.
*   Scalability: The system is designed with scalability in mind, so additional services can be added by extending NGINX rules and updating HAProxy configurations.
*   Security Considerations: Implement firewall rules and access controls between different segments of the network, particularly where sensitive data is involved.

## Troubleshooting Tips

*   If a service isn't reachable, verify that:
    *   NGINX path routing is correctly configured.
    *   HAProxy is properly balancing between available database instances.
    *   Containers are connected to the right Docker network.
*   For streaming issues, ensure that the RTMP server is active and that containers are correctly publishing streams.
