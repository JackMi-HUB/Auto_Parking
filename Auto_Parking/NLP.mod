param Nfe == 200;
param w_penalty == 100000.000000;
param w_a == 0.025000;
param w_w == 0.005100;
param w_phy == 0.000840;
var tf >= 0.1;
var hi = tf / Nfe;
set I := {1..Nfe};
param CC{i in I, j in {1..8}};
param BV{i in {1..6}};

param amax == 1.0;
param vmax == 2.5;
param wmax == 0.5;
param phymax == 0.7;

param Lfc == 2.5877;
param Lrc == 0.2432;
param L_wheelbase == 2.8;

var x{i in I};
var y{i in I};
var theta{i in I};
var v{i in I};
var a{i in I};
var phy{i in I};
var w{i in I};

var xf{i in I};
var yf{i in I};
var xr{i in I};
var yr{i in I};

minimize objective_:
tf + w_a * sum{i in I}(a[i]^2)+ w_phy * sum{i in I}(phy[i]^2)+ w_w * sum{i in I}(w[i]^2) +
w_penalty * (sum{i in {2..Nfe}}((x[i] - x[i-1] - hi * v[i] * cos(theta[i]))^2 + (y[i] - y[i-1] - hi * v[i] * sin(theta[i]))^2 + (v[i] - v[i-1] - hi * a[i])^2 + (theta[i] - theta[i-1] - hi * tan(phy[i]) * v[i] / L_wheelbase)^2 + (phy[i] - phy[i-1] - hi * w[i])^2 + (xf[i] - x[i] - Lfc * cos(theta[i]))^2 + (yf[i] - y[i] - Lfc * sin(theta[i]))^2 + (xr[i] - x[i] - Lrc * cos(theta[i]))^2 + (yr[i] - y[i] - Lrc * sin(theta[i]))^2) + (sin(theta[Nfe]) - sin(BV[6]))^2 + (cos(theta[Nfe]) - cos(BV[6]))^2);


s.t. time_limit:
tf <= 50;


############# Collision Avoidance #############
s.t. Rear_disc_x {i in I}:
CC[i,1] <= xr[i] <= CC[i,2];
s.t. Rear_disc_y {i in I}:
CC[i,3] <= yr[i] <= CC[i,4];
s.t. Front_disc_x {i in I}:
CC[i,5] <= xf[i] <= CC[i,6];
s.t. Front_disc_y {i in I}:
CC[i,7] <= yf[i] <= CC[i,8];

############# Two-Point Boundary Values #############
s.t. EQ_init_x :
x[1] = BV[1];
s.t. EQ_init_y :
y[1] = BV[2];
s.t. EQ_init_theta :
theta[1] = BV[3];
s.t. EQ_init_phy :
phy[1] = 0;
s.t. EQ_init_w :
w[1] = 0;
s.t. EQ_init_v :
v[1] = 0;
s.t. EQ_init_a :
a[1] = 0;
s.t. EQ_end_x :
x[Nfe] = BV[4];
s.t. EQ_end_y :
y[Nfe] = BV[5];
s.t. EQ_end_phy :
phy[Nfe] = 0;
s.t. EQ_end_w :
w[Nfe] = 0;
s.t. EQ_end_v :
v[Nfe] = 0;
s.t. EQ_end_a :
a[Nfe] = 0;

s.t. Bonds_phy {i in I}:
-phymax <= phy[i] <= phymax;
s.t. Bonds_a {i in I}:
-amax <= a[i] <= amax;
s.t. Bonds_v {i in I}:
-vmax <= v[i] <= vmax;
s.t. Bonds_w {i in I}:
-wmax <= w[i] <= wmax;

data;
param: BV := include BV;
param: CC := include CC;