"""
lineadebug.py
=============
Dibuja la ruta planificada por A* como una línea verde sobre el piso de la arena en Webots.
"""

def dibujar_ruta_3d(supervisor, waypoints):
    if not waypoints:
        return

    Z_FLOOR = 0.02  

    coord_parts = []
    # El controlador refactorizado ahora envía tuplas en el plano (X, Y)
    for (cx, cy) in waypoints:
        vrml_x = cx
        vrml_y = cy
        vrml_z = Z_FLOOR 
        coord_parts.append(f"{vrml_x:.4f} {vrml_y:.4f} {vrml_z:.4f}")

    coord_str = ", ".join(coord_parts)
    index_str = ", ".join(str(i) for i in range(len(waypoints))) + ", -1"

    # Nodo Shape de Webots con IndexedLineSet (Transform 0 0 0 se alinea al RectangleArena)
    vrml_string = f"""Transform {{
  translation 0 0 0
  children [
    Shape {{
      appearance Appearance {{
        material Material {{
          emissiveColor 0 1 0
        }}
      }}
      geometry IndexedLineSet {{
        coord Coordinate {{
          point [ {coord_str} ]
        }}
        coordIndex [ {index_str} ]
      }}
    }}
  ]
}}"""

    root_node      = supervisor.getRoot()
    children_field = root_node.getField("children")
    
    # Importa la línea al árbol de nodos de Webots
    children_field.importMFNodeFromString(-1, vrml_string)

    p0x, p0y = waypoints[0]
    pNx, pNy = waypoints[-1]

    print("🟢 Ruta dibujada en el entorno (Sistema ENU)")
    print(f"   Nodos        : {len(waypoints)}")
    print(f"   Inicio (VRML): X={p0x:+.3f}  Y={p0y:+.3f}  Z={Z_FLOOR}")
    print(f"   Meta   (VRML): X={pNx:+.3f}  Y={pNy:+.3f}  Z={Z_FLOOR}")

def checkDebug(node):
    try:
        supervisor_field = node.getField("supervisor")
        DEBUG = supervisor_field is not None and supervisor_field.getSFBool()
    except Exception:
        DEBUG = False

    if DEBUG:
        rot = node.getField("rotation")
        print("[DEBUG] Modo supervisor activo — visualización de ruta habilitada.")
    else:
        print("[INFO] Modo supervisor inactivo — visualización deshabilitada.")
    
    return DEBUG