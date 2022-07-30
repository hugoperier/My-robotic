from Buzzer import Buzzer 
import time

russian_anthem = [
  {'note': "C4",'time': 416},

  {'note': "F4" ,'time': 832},
  {'note': "C4" ,'time': 624},
  {'note': "D4" ,'time': 208},
  {'note': "E4" ,'time': 832},
  {'note': "A3" ,'time': 416},
  {'note': "A3" ,'time': 416},

  {'note': "D4" ,'time': 832},
  {'note': "C4" ,'time': 624},
  {'note': "A#3" ,'time': 208},
  {'note': "C4" ,'time': 832},
  {'note': "F3" ,'time': 416},
  {'note': "F3" ,'time': 416},

  {'note': "G3" ,'time': 832},
  {'note': "G3" ,'time': 416},
  {'note': "A3" ,'time': 416},
  {'note': "A#3" ,'time': 832},
  {'note': "A#3" ,'time': 416},
  {'note': "C4" ,'time': 416},

  {'note': "D4" ,'time': 832},
  {'note': "E4" ,'time': 416},
  {'note': "F4" ,'time': 416},
  {'note': "G4" ,'time': 1664},

  {'note': "A4" ,'time': 832},
  {'note': "G4" ,'time': 624},
  {'note': "F4" ,'time': 208},
  {'note': "G4" ,'time': 832},
  {'note': "E4" ,'time': 416},
  {'note': "C4" ,'time': 416},

  {'note': "F4" ,'time': 832},
  {'note': "E4" ,'time': 624},
  {'note': "D4" ,'time': 208},
  {'note': "E4" ,'time': 832},
  {'note': "A3" ,'time': 416},
  {'note': "A3" ,'time': 416},

  {'note': "D4" ,'time': 832},
  {'note': "C4" ,'time': 624},
  {'note': "A#3" ,'time': 208},
  {'note': "C4" ,'time': 832},
  {'note': "F3" ,'time': 624},
  {'note': "F3" ,'time': 208},

  {'note': "F4" ,'time': 832},
  {'note': "E4" ,'time': 624},
  {'note': "D4" ,'time': 208},
  {'note': "C4" ,'time': 416},
  {'note': "E4" ,'time': 416},
  {'note': "F4" ,'time': 416},
  {'note': "G4" ,'time': 416},

  {'note': "A4" ,'time': 1664},
  {'note': "G4" ,'time': 416},
  {'note': "F4" ,'time': 416},
  {'note': "E4" ,'time': 416},
  {'note': "F4" ,'time': 416},

  {'note': "G4" ,'time': 1248},
  {'note': "C4" ,'time': 416},
  {'note': "C4" ,'time': 416},
  {'note': "E4" ,'time': 416},
  {'note': "F4" ,'time': 416},
  {'note': "G4" ,'time': 416},

  {'note': "F4" ,'time': 1664},
  {'note': "E4" ,'time': 416},
  {'note': "D4" ,'time': 416},
  {'note': "C4" ,'time': 416},
  {'note': "D4" ,'time': 416},

  {'note': "E4" ,'time': 1248},
  {'note': "A3" ,'time': 416},
  {'note': "A3" ,'time': 416},
  {'note': "C4" ,'time': 416},
  {'note': "D4" ,'time': 416},
  {'note': "E4" ,'time': 416},

  {'note': "F4" ,'time': 832},
  {'note': "D4" ,'time': 624},
  {'note': "E4" ,'time': 208},
  {'note': "F4" ,'time': 832},
  {'note': "D4" ,'time': 624},
  {'note': "E4" ,'time': 208},

  {'note': "F4" ,'time': 832},
  {'note': "D4" ,'time': 416},
  {'note': "F4" ,'time': 416},
  {'note': "A#4" ,'time': 1664},

  {'note': "A#4" ,'time': 1664},
  {'note': "A4" ,'time': 416},
  {'note': "G4" ,'time': 416},
  {'note': "F4" ,'time': 416},
  {'note': "G4" ,'time': 416},

  {'note': "A4" ,'time': 1248},
  {'note': "F4" ,'time': 416},
  {'note': "F4" ,'time': 1664},

  {'note': "G4" ,'time': 1664},
  {'note': "F4" ,'time': 416},
  {'note': "E4" ,'time': 416},
  {'note': "D4" ,'time': 416},
  {'note': "E4" ,'time': 416},

  {'note': "F4" ,'time': 1248},
  {'note': "D4" ,'time': 416},
  {'note': "D4" ,'time': 1664},

  {'note': "F4" ,'time': 832},
  {'note': "E4" ,'time': 624},
  {'note': "D4" ,'time': 208},
  {'note': "C4" ,'time': 832},
  {'note': "F3" ,'time': 832},

  {'note': "F4" ,'time': 832},
  {'note': "E4" ,'time': 624},
  {'note': "D4" ,'time': 208},
  {'note': "C4" ,'time': 1248},
  {'note': "C4" ,'time': 416},

  {'note': "C4" ,'time': 832},
  {'note': "D4" ,'time': 416},
  {'note': "E4" ,'time': 416},
  {'note': "F4" ,'time': 1664}
]

if __name__ == 'main':
  buzzer = Buzzer(23)

  for i in russian_anthem:
      print(i["note"])
      try:
          buzzer.play(i["note"], i['time'] / 1000)
      except Exception:
          print(i["note"] + " not working")