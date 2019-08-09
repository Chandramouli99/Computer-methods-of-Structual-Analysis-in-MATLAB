%input
clc;
clear;
n=2; % number of members
I=[1.5 1]; %Moment of inertia in m4
L=[4 2]; %length in m
A=[1.25 1]; %Area in m2
uu=3; %Number of unrestrained degrees of freedom
ur=6; %Number of restrained degrees of freedom 
uul=[1 2 3];
url=[4 5 6 7 8 9];
l1=[7 1 9 2 8 9];
l2=[1 4 2 5 3 6];
l=[l1; l2];
dof= uu + ur;
Ktotal=zeros (dof);
Tt1=zeros (6);
Tt2=zeros(6);
fem1=[0;0;0;0;0;0];
fem2=[40;-40;40;40;0;0];
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
    fprintf('Transformation matrix Transpose, [T]=\n');
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
        end
end
fprintf('Stiffness Matrix of complete structure, Ktotal =\n');
disp(Ktotal);
Kunr=zeros(3);
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
jl=[-40;-40;15;40;-40;0;0;0;0];
jlu=[0;0;10];
delu=KuuInv*jlu;
fprintf('Joint Load vector, Jl =\n');
disp(jl);
fprintf('Urestrained displacements,=\n');
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
    end
end
        
        


