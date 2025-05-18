#!/bin/bash
# Start the ASTRA Playground server

# Default port
PORT=12000

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --port)
      PORT="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [--port PORT]"
      exit 1
      ;;
  esac
done

# Start the server
echo "Starting ASTRA Playground on port $PORT..."
python3 "$(dirname "$0")/server.py" --port "$PORT" --bind 0.0.0.0