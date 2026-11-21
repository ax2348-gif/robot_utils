import json,numpy as np,math,sys
r=json.load(open(sys.argv[1]))['camera']; T=np.array(r['base_to_color'])
fx=320/math.tan(math.radians(r['hfov_deg']/2)); fy=240/math.tan(math.radians(r['vfov_deg']/2))
def ray(u,v,z):
  d=np.array([(u-320)/fx,(v-240)/fy,1.0]); w=T[:3,:3]@d; o=T[:3,3]; s=(z-o[2])/w[2]; return o+s*w
z=float(sys.argv[2])
for p in sys.argv[3:]:
  u,v=map(float,p.split(',')); print(p, np.round(ray(u,v,z),3).tolist())
