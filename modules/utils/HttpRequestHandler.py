from http.server import BaseHTTPRequestHandler, HTTPServer

class HttpRequestHandler(BaseHTTPRequestHandler):
    def __init__(self, *args):
        get = args[0]
        post = args[1]
        self.routes = {
            "GET": get,
            "POST": post
        }
        BaseHTTPRequestHandler.__init__(self, *args[2:])

    # def add_route(self, method, path, handler):
    #     if method not in self.routes:
    #         raise Exception("Invalid method")
    #     if path in self.routes[method]:
    #         raise Exception("Route already exists")
    #     self.routes[method][path] = handler

    def do_GET(self):
        print("GET", self.path)
        if self.path in self.routes["GET"]:
            self.routes["GET"][self.path](self)
        else:
            self.not_found()
        
    def do_POST(self):
        print("POST", self.path)
        if self.path in self.routes["POST"]:
            self.routes["POST"][self.path](self)
        else:
            self.not_found()
    
    def send_json(self, data):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def success(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

    def not_found(self):
        self.send_response(404)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
    
    def send_error(self, code, message):
        self.send_response(code)
        if message:
            self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        if message:
            self.wfile.write(message.encode())

    