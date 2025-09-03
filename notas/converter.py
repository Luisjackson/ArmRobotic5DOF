def map_value(value, from_min, from_max, to_min, to_max):
    """Converte um valor de uma faixa para outra."""
    return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

# rad_min, rad_max, tick_min, tick_max
pan_calib     = (-1.57, 1.57, 200, 900)
lift_calib    = (-1.57, 1.57, 165, 865) 
elbow_calib   = (-1.57, 1.57, 300, 563)
wrist_calib   = (-1.57, 1.57, 200, 830) 
gripper_calib = (-0.600, 0.508, 390, 600)

posicao_radianos = [0.0, 0.2, -0.7, -0.1] # pan, lift, elbow, wrist
posicao_garra_ticks = 506

pan_ticks     = map_value(posicao_radianos[0], *pan_calib)
lift_ticks    = map_value(posicao_radianos[1], *lift_calib)
elbow_ticks   = map_value(posicao_radianos[2], *elbow_calib)
wrist_ticks   = map_value(posicao_radianos[3], *wrist_calib)

print("--- Valores em Ticks para a Posição HOME ---")
print(f"Pan:     {int(pan_ticks)}")
print(f"Lift:    {int(lift_ticks)}")
print(f"Elbow:   {int(elbow_ticks)}")
print(f"Wrist:   {int(wrist_ticks)}")
print(f"Gripper: {posicao_garra_ticks}")
print("\nComando para o C++:")
print(f"P {int(pan_ticks)} {int(lift_ticks)} {int(elbow_ticks)} {int(wrist_ticks)} {posicao_garra_ticks}")