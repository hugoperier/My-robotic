from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer, MediaRelay
from aiortc.rtcrtpsender import RTCRtpSender
import time
import av

class WebRTCCamera:
    def __init__(self, config):
        print("initialized")
        self.config = config
        self.width = config['width']
        self.height = config['height']
        self._fps = config["fps"]   
        self.__sessions__ = set()
        self._relay = None
        self._media = None

    def make_player(self):
        if self._media is None:
            options = {"framerate": f"{self._fps}", "video_size": f"{self.width}x{self.height}"}
            self._media = MediaPlayer("/dev/video0", format="v4l2", options=options)
            self._relay = MediaRelay()  
        return self._relay.subscribe(self._media.video)

    async def start_session(self, sdp, sess_type):
        session_description = RTCSessionDescription(sdp=sdp, type=sess_type)
        pc = RTCPeerConnection()
        self.__sessions__.add(pc)
        
        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print("Connection state is %s" % pc.connectionState)
            if pc.connectionState == "failed":
                await pc.close()
                self.__sessions__.discard(pc)
                if (self.__sessions__.__len__() == 0):
                    self._media.video.stop()
                    del self._media
                    self._media = None
                    del self._relay
                    self._relay = None

        video = self.make_player()
        pc.addTrack(video)        
        await pc.setRemoteDescription(session_description)

        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        return {
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type
        }
