using GeometryBasics
using CoordinateTransformations, Rotations
using RobotDynamics
using Colors
using StaticArrays 
using MeshCat
using Blink
using LinearAlgebra
using TrajOptPlots

Base.@kwdef struct KneedWalker <: AbstractModel
    g::Float64  = 9.81
    mh::Float64 = 0.5
    mt::Float64  = 0.5
    ms::Float64  = 0.05
    a1::Float64 = 0.375
    b1::Float64 = 0.125
    a2::Float64 = 0.175
    b2::Float64 = 0.325
    L::Float64  = 1.0 
end
RobotDynamics.state_dim(::KneedWalker) = 6
RobotDynamics.control_dim(::KneedWalker) = 2

function RobotDynamics.dynamics(model::KneedWalker, X, U, t, mode::Integer)
    
    mh,mt,ms = model.mh, model.mt, model.ms
    
    g = model.g
    a1,b1,a2,b2,L = model.a1,model.b1,model.a2,model.b2,model.L

    q1 = wrap2pi(X[1])
    q2 = wrap2pi(X[2])
    q3 = wrap2pi(X[3])
    dq1,dq2,dq3 = X[4:6]
    
    q = X[1:3]
    dq = X[4:6]
    
    if mode == 1 #Unlocked Dynamics
        
        H = get_H_UL(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,g)
        B = get_B_UL(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,dq1,dq2,dq3)
        G = get_G_UL(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,g)
        
        C = zeros(3,2)
        C[2,1] = 1
        C[3,2] = 1

        ddq = -H\(B*dq+G+C*U)

        ẋ = [dq;ddq]
        
    else
        H = get_H_L(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,g)
        B = get_B_L(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,dq1,dq2,dq3)
        G = get_G_L(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,g)
        
        C = zeros(2,2)
        C[2,1] = 1
        ddq = -H\(B*dq[1:2]+G+C*U)
        
#         ẋ = [dq;ddq;ddq[2]]'
        ẋ = [dq;ddq;ddq[2]]
    end
end

function jumpmap(model::KneedWalker, X, mode) 
    mh,mt,ms = model.mh, model.mt, model.ms
    g = model.g
    a1,b1,a2,b2,L = model.a1,model.b1,model.a2,model.b2,model.L
    q1,q2,q3 = X[1:3]
    dq1,dq2,dq3 = X[4:6]
    
     if mode==1 #Knee Strike 
        Qp,Qm = get_Qs_KneeStrike(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3)
        qp = Qp\Qm*dq

            [
                X[1];  X[2];  X[2];
                qp[1]; qp[2]; qp[2]
            ]

    else #Heel Strike
#         Qp,Qm = get_Qs_HeelStrike(mh,mt,ms,a1,a2,b1,b2,L,q2,q1,q1)
#         qp = Qp\(Qm*dq[1:2])
        qp = [-1.1014 -0.0399 -0.0399]

           [
            X[2];   X[1];  X[1];
            qp[1]; qp[2]; qp[2]
          ]
    end  
end

function rk4(model::KneedWalker, x,u,t, dt, mode)
    f1 = dynamics(model, x,u,t, mode)
    f2 = dynamics(model, x + 0.5*dt*f1,u,t, mode)
    f3 = dynamics(model, x + 0.5*dt*f2,u,t, mode)   
    f4 = dynamics(model, x + dt*f3, u,t, mode)
    return x + dt*(f1 + 2*f2 + 2*f3 + f4)/6
end

function set_mesh!(vis,model::KneedWalker)
    r = 0.1
    a1,b1,a2,b2,L = model.a1,model.b1,model.a2,model.b2,model.L
    ls = a1+b1
    lt = a2+b2
    
    #Torso 
    body = Sphere(Point3f0(0,0,0), r)
    setobject!(vis["robot"]["torso"]["body"], body, MeshPhongMaterial(color=colorant"gray"))
    axle = Cylinder(Point3f0(0,0,0), Point3f0(0,r,0), 0.03f0)

    #Upper Leg
    setobject!(vis["robot"]["torso"]["Laxle1"], axle, MeshPhongMaterial(color=colorant"black"))
    setobject!(vis["robot"]["torso"]["Raxle1"], axle, MeshPhongMaterial(color=colorant"black"))
    settransform!(vis["robot"]["torso"]["Laxle1"], Translation(0,+r/2,0))
    settransform!(vis["robot"]["torso"]["Raxle1"], Translation(0,-3r/2,0))
    
    Lthigh = Cylinder(Point3f0(0,+r,-lt), Point3f0(0,+r,0), 0.03f0)
    Rthigh = Cylinder(Point3f0(0,-r,-lt), Point3f0(0,-r,0), 0.03f0)
    setobject!(vis["robot"]["torso"]["Lthigh"]["geom"], Lthigh, MeshPhongMaterial(color=colorant=colorant"green"))
    setobject!(vis["robot"]["torso"]["Rthigh"]["geom"], Rthigh, MeshPhongMaterial(color=colorant=colorant"green"))
    
    #Lower Leg
    setobject!(vis["robot"]["torso"]["Lthigh"]["Laxle2"], axle, MeshPhongMaterial(color=colorant"black"))
    setobject!(vis["robot"]["torso"]["Rthigh"]["Raxle2"], axle, MeshPhongMaterial(color=colorant"black"))
    settransform!(vis["robot"]["torso"]["Lthigh"]["Laxle2"], Translation(0,+r/2,-lt))
    settransform!(vis["robot"]["torso"]["Rthigh"]["Raxle2"], Translation(0,-3r/2,-lt))
    
    Lshank = Cylinder(Point3f0(0,r,-ls), Point3f0(0,r,0), 0.03f0)
    Rshank = Cylinder(Point3f0(0,-r,-ls), Point3f0(0,-r,0), 0.03f0)
    setobject!(vis["robot"]["torso"]["Lthigh"]["Lshank"]["geom"], Lshank, MeshPhongMaterial(color=colorant=colorant"yellow"))
    setobject!(vis["robot"]["torso"]["Rthigh"]["Rshank"]["geom"], Rshank, MeshPhongMaterial(color=colorant=colorant"yellow"))
    
    #Feet
    foot = HyperSphere(Point3f0(0,0,0f0), 0.05f0)
    setobject!(vis["robot"]["torso"]["Lthigh"]["Lshank"]["Lfoot"]["geom"], foot, MeshPhongMaterial(color=colorant"firebrick"))
    setobject!(vis["robot"]["torso"]["Rthigh"]["Rshank"]["Rfoot"]["geom"], foot, MeshPhongMaterial(color=colorant"firebrick"))
    settransform!(vis["robot"]["torso"]["Lthigh"]["Lshank"]["Lfoot"]["geom"], Translation(0,r,-0.5))
    settransform!(vis["robot"]["torso"]["Rthigh"]["Rshank"]["Rfoot"]["geom"], Translation(0,-r,-0.5))
end

function TrajOptPlots.visualize!(vis, model::KneedWalker, X,side_mode)
    q1,q2,q3 = X[1],X[2],X[3]
    if side_mode==1
        settransform!(vis["robot"]["torso"], Translation(0,0,1))
        settransform!(vis["robot"]["torso"]["Lthigh"],LinearMap(RotY(q1)))
        settransform!(vis["robot"]["torso"]["Rthigh"],LinearMap(RotY(q2)))
        settransform!(vis["robot"]["torso"]["Lthigh"]["Lshank"],Translation(0,0,-0.5))
        settransform!(vis["robot"]["torso"]["Rthigh"]["Rshank"],compose(Translation(0,0,-0.5),LinearMap(RotY(q3-q2))))
    else
        settransform!(vis["robot"]["torso"], Translation(0,0,1))
        settransform!(vis["robot"]["torso"]["Rthigh"],LinearMap(RotY(q1)))
        settransform!(vis["robot"]["torso"]["Lthigh"],LinearMap(RotY(q2)))
        settransform!(vis["robot"]["torso"]["Rthigh"]["Rshank"],Translation(0,0,-0.5))
        settransform!(vis["robot"]["torso"]["Lthigh"]["Lshank"],compose(Translation(0,0,-0.5),LinearMap(RotY(q3-q2))))
    end
end

function stance1_dynamics(model::KneedWalker, X, U)
    #Foot 1 is in contact
    mh,mt,ms = model.mh, model.mt, model.ms
    g = model.g
    a1,b1,a2,b2,L = model.a1,model.b1,model.a2,model.b2,model.L

    q1 = wrap2pi(X[1])
    q2 = wrap2pi(X[2])
    q3 = wrap2pi(X[3])
    
    dq1,dq2,dq3 = X[4:6]
    
    q = X[1:3]
    dq = X[4:6]
            
    H = get_H_UL(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,g)
    B = get_B_UL(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,dq1,dq2,dq3)
    G = get_G_UL(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,g)

    C = zeros(3,2)
    C[2,1] = 1
    C[3,2] = 1

    ddq = -H\(B*dq+G+C*U)
    
    ẋ = [dq;ddq]
end

function stance2_dynamics(model::KneedWalker, X, U)
     mh,mt,ms = model.mh, model.mt, model.ms

    g = model.g
    a1,b1,a2,b2,L = model.a1,model.b1,model.a2,model.b2,model.L

    q1 = wrap2pi(X[1])
    q2 = wrap2pi(X[2])
    q3 = wrap2pi(X[3])
    
    dq1,dq2,dq3 = X[4:6]
    
    q = X[1:3]
    dq = X[4:6]
    
    H = get_H_L(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,g)
    B = get_B_L(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,dq1,dq2,dq3)
    G = get_G_L(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,g)

    C = zeros(2,2)
    C[2,1] = 1
    ddq = -H\(B*dq[1:2]+G+C*U)
    
    ẋ = [dq;ddq;ddq[2]]
end

function stance1_dynamics_rk4(model, x,u, h)
    #RK4 integration with zero-order hold on u
    f1 = stance1_dynamics(model, x, u)
    f2 = stance1_dynamics(model, x + 0.5*h*f1, u)
    f3 = stance1_dynamics(model, x + 0.5*h*f2, u)
    f4 = stance1_dynamics(model, x + h*f3, u)
    return x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
end

function stance2_dynamics_rk4(model, x,u, h)
    #RK4 integration with zero-order hold on u
    f1 = stance2_dynamics(model, x, u)
    f2 = stance2_dynamics(model, x + 0.5*h*f1, u)
    f3 = stance2_dynamics(model, x + 0.5*h*f2, u)
    f4 = stance2_dynamics(model, x + h*f3, u)
    return x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
end

function stance1_jacobian(model::KneedWalker, x, u,dt)
    xi = SVector{6}(1:6)
    ui = SVector{2}(1:2) .+ 6
    f(z) = stance1_dynamics_rk4(model, z[xi], z[ui],dt)
    ForwardDiff.jacobian(f, [x; u])
end

function stance2_jacobian(model::KneedWalker, x, u, dt)
    xi = SVector{6}(1:6)
    ui = SVector{2}(1:2) .+ 6
    f(z) = stance2_dynamics_rk4(model, z[xi], z[ui],dt)
    ForwardDiff.jacobian(f, [x; u])
end


function jump1_map(X)
    #Foot 1 experiences inelastic collision
    Qmin = zeros(eltype(X),2,3)
    Qplus = zeros(eltype(X),2,2)
    qp = zeros(eltype(X),2)
    xn = zeros(eltype(X),6)
    mh,mt,ms = model.mh, model.mt, model.ms
    g = model.g
    a1,b1,a2,b2,L = model.a1,model.b1,model.a2,model.b2,model.L
    q1,q2,q3 = X[1:3]
    dq1,dq2,dq3 = X[4:6]
#     println(dq)
#     dq = X[4:6]
#     println(dq)
    
    lt = a2+b2
    ls = a1+b1
    
    alpha = cos(q1-q2)
    beta = cos(q1-q3)
    gamma = cos(q2-q3)
    
#     Qmin = zeros(eltype(X),2,3)
#     Qplus = zeros(eltype(X),2,2)
    
    Qmin[1,1] = -(ms*lt+mt*b2)*L*cos(alpha) - ms*b1*L*cos(beta) + (mt+ms+mh)*L^2+ms*a1^2+mt*(ls+a2)^2
    Qmin[1,2] = -(ms*lt+mt*b2)*L*cos(alpha)+ms*b1*lt*cos(gamma)+mt*b2^2+ms*lt^2
    Qmin[1,3] = -ms*b1*L*cos(beta)+ms*b1*lt*cos(gamma)+ms*b1^2
    Qmin[2,1] = -(ms*lt+mt*b2)*L*cos(alpha)-ms*b1*L*cos(beta)
    Qmin[2,2] = ms*b1*lt*cos(gamma)+ms*lt^2+mt*b2^2
    Qmin[2,3] = ms*b1*lt*cos(gamma)+ms*b1^2
    
    Qplus[2,1] = -(ms*(b1+lt)+mt*b2)*L*cos(alpha)
    Qplus[1,1] = Qplus[2,1]+mt*(ls+a2)^2+(mh+mt+ms)*L^2+ms*a1^2
    Qplus[1,2] = Qplus[2,1] + ms*(lt+b1)^2+mt*b2^2
    Qplus[2,2] = ms*(lt+b1)^2+mt*b2^2

    qp .= Qplus\Qmin*dq

    xn .= [
                X[1];  X[2];  X[2];
                qp[1]; qp[2]; qp[2]
          ]
    return xn
end

function jump2_map(X)
    #Foot 2 experiences inelastic collision
    mh,mt,ms = model.mh, model.mt, model.ms
    g = model.g
    a1,b1,a2,b2,L = model.a1,model.b1,model.a2,model.b2,model.L
    q1,q2,q3 = X[1:3]
    dq1,dq2,dq3 = X[4:6]
#     dq = X[4:6]

#     Qp,Qm = get_Qs_HeelStrike(mh,mt,ms,a1,a2,b1,b2,L,q2,q1,q1)
#     qp = Qp\(Qm*dq[1:2])
    qp = [-1.1014 -0.0399 -0.0399]

    xn =  [
            X[2];   X[1];  X[1];
            qp[1]; qp[2]; qp[2]
          ]

    return xn
end

function jump1_jacobian(x)
    J = ForwardDiff.jacobian(jump1_map,x)
    return J
end 

function jump2_jacobian(x)
# HERE THE ENDING IS 0 BECAUSE IT IS DEFINED AS CONSTANTS, NEED TO CHANGE THIS IF NOT WORKING
    J = ForwardDiff.jacobian(jump2_map,x)
    return J
end 

function get_Qs_KneeStrike(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3)
    
    lt = a2+b2
    ls = a1+b1
    
    alpha = cos(q1-q2)
    beta = cos(q1-q3)
    gamma = cos(q2-q3)
    
    Qmin = zeros(2,3)
    Qplus = zeros(2,2)
    
    Qmin[1,1] = -(ms*lt+mt*b2)*L*cos(alpha) - ms*b1*L*cos(beta) + (mt+ms+mh)*L^2+ms*a1^2+mt*(ls+a2)^2
    Qmin[1,2] = -(ms*lt+mt*b2)*L*cos(alpha)+ms*b1*lt*cos(gamma)+mt*b2^2+ms*lt^2
    Qmin[1,3] = -ms*b1*L*cos(beta)+ms*b1*lt*cos(gamma)+ms*b1^2
    Qmin[2,1] = -(ms*lt+mt*b2)*L*cos(alpha)-ms*b1*L*cos(beta)
    Qmin[2,2] = ms*b1*lt*cos(gamma)+ms*lt^2+mt*b2^2
    Qmin[2,3] = ms*b1*lt*cos(gamma)+ms*b1^2
    
    Qplus[2,1] = -(ms*(b1+lt)+mt*b2)*L*cos(alpha)
    Qplus[1,1] = Qplus[2,1]+mt*(ls+a2)^2+(mh+mt+ms)*L^2+ms*a1^2
    Qplus[1,2] = Qplus[2,1] + ms*(lt+b1)^2+mt*b2^2
    Qplus[2,2] = ms*(lt+b1)^2+mt*b2^2
    
    return Qplus,Qmin
        
end


function get_Qs_HeelStrike(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3)
    
    lt = a2+b2
    ls = a1+b1
    
    alpha = cos(q1-q2)
    
    Qmin = zeros(2,2)
    Qplus = zeros(2,2)
    
    Qmin[1,2] = -ms*a1*(lt+b1)+mt*b2*(ls+a2)
    Qmin[1,1] = Qmin[1,2] + (mh*L+2*mt*(a2+ls)+ms*a1)*L*cos(alpha)
    Qmin[2,1] = Qmin[1,2]
    Qmin[2,2] = 0.0
    
    Qplus[2,1] = -(ms*(b1+lt)+mt*b2)*L*cos(alpha)
    Qplus[1,1] = Qplus[2,1] + (ms+mt+mh)*L^2+ms*a1^2+mt*(a2+ls)^2
    Qplus[1,2] = Qplus[2,1]+ms*(b1+lt)^2+mt*b2^2
    Qplus[2,2] = ms*(lt+b1)^2+mt*b2^2
    
    return Qplus,Qmin
        
end

    
function get_H_UL(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,g)
    lt = a2+b2
    ls = a1+b1
    
    H11 = ms*(a1^2) + mt*(ls+a2)^2 + (mh+ms+mt)*(L^2)
    H12 = -(mt*b2+ms*lt)*L*cos(q2-q1)
    H13 = -ms*b1*L*cos(q3-q1)
    H22 = mt*b2^2+ms*lt^2
    H23 = ms*lt*b1*cos(q3-q2)
    H33 = ms*b1^2
    
    return [H11 H12 H13;
            H12 H22 H23;
            H13 H23 H33]
end

function get_B_UL(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,dq1,dq2,dq3)
    lt = a2+b2
    ls = a1+b1

    h122 = -(mt*b2+ms*lt)*L*sin(q1-q2)
    h133 = -ms*b1*L*sin(q1-q3)
    h211 = -h122
    h233 = ms*lt*b1*sin(q3-q2)
    h311 = -h133
    h322 = -h233
    
    return [0 h122*dq2 h133*dq3;
            h211*dq1 0 h233*dq3;
            h311*dq1 h322*dq2 0]
end

function get_G_UL(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,g)
    lt = a2+b2
    ls = a1+b1
    
    G1 = -(ms*a1+mt*(ls+a2)+L*(mh+ms+mt))*g*sin(q1)
    G2 = (mt*b2+ms*lt)*g*sin(q2)
    G3 = ms*b1*g*sin(q3)
    
    return [G1;
            G2;
            G3]
end

function get_H_L(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,g)
    lt = a2+b2
    ls = a1+b1
    
    H11 = ms*a1^2+mt*(ls+a2)^2+(mh+ms+mt)*L^2
    H12 = -(mt*b2+ms*(lt+b1))*L*cos(q2-q1)
    H22 = mt*b2^2+ms*(lt+b1)^2
    
    return [H11 H12 ;
            H12 H22 ]
end

function get_B_L(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,dq1,dq2,dq3)
    lt = a2+b2
    ls = a1+b1

    h = -(mt*b2+ms*(lt+b1))*L*sin(q1-q2)
    
    return [0 h*dq2;
            -h*dq1 0]
end

function get_G_L(mh,mt,ms,a1,a2,b1,b2,L,q1,q2,q3,g)
    lt = a2+b2
    ls = a1+b1
    
    G1 = -(ms*a1+mt*(ls+a2)+(mh+mt+ms)*L)*g*sin(q1)
    G2 = (mt*b2+ms*(lt+b1))*g*sin(q2)
    
    return [G1;
            G2]
end



