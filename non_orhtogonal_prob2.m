%input
clc;
clear;
n=3; % number of members
I=[2.28E-3 3.125E-3 2.28E-3]; %Moment of inertia in m4
L=[4 6 4.472]; %length in m
A=[0.135 0.15 0.135]; %Area in m2
theta=[90 0 -63.435];  %angle in degrees
uu=6; %Number of unrestrained degrees of freedom
ur=6; %Number of restrained degrees of freedom
uul=[1 2 3 4 5 6];
url=[7 8 9 10 11 12];
l1=[7 1 10 4 9 3];
l2=[1 2 4 6 3 5];
l3=[2 8 6 12 5 11];
l=[l1; l2; l3];
dof= uu + ur;
Ktotal=zeros (dof);
Tt1=zeros (6);
Tt2=zeros(6);
fem1=[0;0;0;0;0;0];
fem2=[0;0;0;0;0;0];
fem3=[0;0;0;0;0;0];
%% rotation coefficient for each member
rc1 =4.*I./L;
rc2 =2.*I./L;
rc3=A./L;
cx=cosd(theta);
cy=sind(theta);
%%stiffness matrix
for i=1:n
    Knew =zeros(dof);
    k1=[rc1(i); rc2(i); (rc1(i)+rc2(i))/L(i); (-(rc1(i)+rc2(i))/L(i)); 0; 0];
    k2=[rc2(i); rc1(i); (rc1(i)+rc2(i))/L(i); (-(rc1(i)+rc2(i))/L(i)); 0; 0];
    k3=[(rc1(i)+rc2(i))/L(i); (rc1(i)+rc2(i))/L(i); (2*(rc1(i)+rc2(i))/(L(i)^2));(-2*(rc1(i)+rc2(i))/(L(i)^2)); 0;0];
    k4=-k3;
    k5=[0;0;0;0;rc3(i);-rc3(i)];
    k6=[0;0;0;0;-rc3(i);rc3(i)];
    K=[k1 k2 k3 k4 k5 k6];
    fprintf('Member Number=');
    disp(i);
    fprintf('Local stiffness matrix of member, [k]=\n');
    disp(K);
    T1=[1;0;0;0;0;0];
    T2=[0;1;0;0;0;0];
    T3=[0;0;cx(i);0;cy(i); 0];
    T4=[0;0;0;cx(i);0;cy(i)];
    T5=[0;0;-cy(i);0;cx(i);0];
    T6=[0;0;0;-cy(i);0;cx(i)];
    T=[T1 T2 T3 T4 T5 T6];
    fprintf('Transformation of member, [T]=n\n');
    disp(T);
    Ttr=T';
    fprintf('Transformation of Transpose, [T]=\n');
    disp(Ttr);
    Kg=Ttr*K*T;
    fprintf('Global Matrix, [K global]=\n');
    disp(Kg);
    for p=1:6
        for q=1:6
            Knew((l(i,p)),(l(i,q)))=Kg(p,q);
        end
    end
    Ktotal=Ktotal + Knew;
        if i==1
            Tr1=T;
            Kg1=Kg;
            fembar1=Tt1'*fem1;
        elseif i==2
            Tt2=T;
            Kg2=Kg;
            fembar2=Tt2'*fem2;
        elseif i==3
            Tt3=T;
            Kg3=Kg;
            fembar3=Tt3'*fem3;
        end
end
fprintf('Stiffness Matrix of complete structure, Ktotal =\n');
disp(Ktotal);
Kunr=zeros(6);
for x=1:uu
    for y=1:uu
        Kunr(x,y)=Ktotal (x,y);
    end
end
fprintf('unrestraied Stiffness of sub-matrix, [Kuu]=\n');
disp(Kunr);
KuuInv=inv(Kunr);
fprintf('Inverse of unrestrained sub-matrix, Kuu inverse=\n');
disp(KuuInv);
%% Creation of Joint vector
jl=[0;0;50;0;0;-100;0;0;0;0;0;0];
jlu=[0;0;50;0;0;-100];
delu=KuuInv*jlu;
fprintf('Joint Load vector, Jl =\n');
disp(jl);
fprintf('displacements are Delu =\n');
disp(delu);
delr=[0;0;0;0;0;0];
del=zeros(dof,1);
del=[delu;delr];
deli=zeros(6,1);
for i=1:n
    for p=1:6
        deli(p,1)=del((l(i,p)),1);
    end
    if i==1 
        delbar1=deli;
        mbar1=(Kg1*delbar1)+fembar1;
        fprintf('Member Number=');
        disp(i);
        fprintf('Global displacemant matrix [DeltaBar]=\n');
        disp(delbar1);
        fprintf('Global End moment matrix=\n');
        disp(mbar1);
    elseif i==2
        delbar2=deli;
        mbar2=(Kg2*delbar2)+fembar2;
        fprintf('Member Number=');
        disp(i);
        fprintf('Global displacemant matrix [DeltaBar]=\n');
        disp(delbar2);
        fprintf('Global End moment matrix=\n');
        disp(mbar2);
    elseif i==3
        delbar3=deli;
        mbar3=(Kg3*delbar3)+fembar3;
        fprintf('Member Number=');
        disp(i);
        fprintf('Global displacemant matrix [DeltaBar]=\n');
        disp(delbar3);
        fprintf('Global End moment matrix=\n');
        disp(mbar3);
    end
end
        
        


