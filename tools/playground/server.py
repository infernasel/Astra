#!/usr/bin/env python3
"""
Simple HTTP server for the ASTRA Language Playground
"""

import http.server
import socketserver
import os
import argparse

class ASTRAPlaygroundHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=os.path.dirname(os.path.abspath(__file__)), **kwargs)

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'X-Requested-With, Content-Type, Accept')
        super().end_headers()

def run_server(port=12000, bind="0.0.0.0"):
    handler = ASTRAPlaygroundHandler
    
    with socketserver.TCPServer((bind, port), handler) as httpd:
        print(f"ASTRA Playground server running at http://{bind}:{port}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer stopped.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ASTRA Language Playground Server")
    parser.add_argument("--port", type=int, default=12000, help="Port to run the server on")
    parser.add_argument("--bind", type=str, default="0.0.0.0", help="Address to bind the server to")
    
    args = parser.parse_args()
    run_server(port=args.port, bind=args.bind)