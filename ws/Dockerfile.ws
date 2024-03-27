# Use the official Golang image as the base image
FROM golang:1.21 AS builder

# Set the working directory inside the container
WORKDIR /application

# Copy the rest of the application source code into the container
COPY ws/app/ .
COPY ws/entrypoint.ws.sh .

# Download Go module dependencies
RUN go mod download

# Build the Go application inside the container
RUN CGO_ENABLED=0 GOOS=linux go build -o /application/app ./src

# PHASE 2

# Use a minimal base image to reduce the container size
FROM alpine:latest

# Set the working directory inside the container
WORKDIR /application

# Copy the built Go application from the builder stage to the final image
COPY --from=builder /application/app .
COPY --from=builder /application/entrypoint.ws.sh .

ENTRYPOINT ["/bin/sh", "./entrypoint.ws.sh"]

