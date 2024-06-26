---
title: Trajectory Smoothing
date: 2024-05-07 12:03:00 +0800
categories: [Optimal Control]
tags: [lqr servo]
author: Sidney Lin
math: true
comments: true
mermaid: true
image: /assets/2024-05-07-Trajectory-Smoothing.assets/trajectory_smoothing.jpg
---

## Error State Incremental Trajectory Model Derivation
Starting with a time-varying non-linear trajectory dynamics

$$
\dot{x} = f(t,x,u)
$$

Recall the linearization section described in [lqr post](/posts/Linear-Quadratic-Optimal-Tracking-for-Time-Varying-Nonlinear-System/), the continuous-time error space linear system could be expressed as 

$$
\begin{aligned}
\dot{x} &= A(x-x_l) + B(u-u_l) + f(t,x_l,u_l) \\
(\dot{x} - \dot{x_l}) &= A(x-x_l) + B(u-u_l) + [f(t,x_l,u_l) - \dot{x_l}]
\end{aligned}
$$

where

$$
\begin{aligned}
A &= \left.\frac{\partial f}{\partial x}\right \vert_{(x_l,u_l)} \\
B &= \left.\frac{\partial f}{\partial u}\right \vert_{(x_l,u_l)} \\
\end{aligned}
$$

to further simplify the notation, define error state as

$$
\begin{aligned}
\hat{x} &= x - x_l \\
\hat{u} &= u - u_l \\
w &= f(t,x_l,u_l) - \dot{x_l}
\end{aligned}
$$

then, the continuous-time error space system is

$$
\begin{aligned}
\dot{\hat{x}} = A\hat{x}+B\hat{u}+w
\end{aligned}
$$

note that if $(x_l,u_l)$ is generated by system function $\dot{x} = f(t,x,u)$, $w = 0$.

Recall the discretization section described in [mpc post](/posts/Model-Predictive-Control-Part-1-Generized-Problem-Setup), the discrete-time error space linear system could be expressed as 

$$
\begin{aligned}
\hat{x}_{k+1} = A_d\hat{x}_k+B_d\hat{u}_k+w_k
\end{aligned}
$$

where 

$$
\begin{aligned}
A_d &= e^{At}\\
B_d &= \int_{0}^{t}e^{Av}dvB\\
w_k &= \int_{0}^{t}e^{Av}dvw
\end{aligned}
$$

note that if $A$ is invertible, we have

$$
\begin{aligned}
\int_{0}^{t}e^{Av}dv = \left. A^{-1}e^{Av} \right \vert_{0}^{t} = A^{-1}(e^{At} - I)
\end{aligned}
$$

otherwise, $e^{At}$ could be approximated by Taylor expansion

$$
\begin{aligned}
e^{At} = \sum_{k=0}^{\infty}\frac{1}{k!}(At)^k
\end{aligned}
$$

this yields

$$
\begin{aligned}
\int_{0}^{t}e^{Av}dv 
&= \int_{0}^{t}\sum_{k=0}^{\infty}\frac{1}{k!}(Av)^kdv \\
&= \sum_{k=0}^{\infty}\frac{1}{k!}A^{k}\int_{0}^{t}v^{k}dv \\
&= \sum_{k=0}^{\infty}\frac{1}{k!}A^{k}\left. \frac{v^{k+1}}{k+1}\right \vert_{0}^{t} \\
&= \sum_{k=0}^{\infty}\frac{1}{k+1!}A^{k}t^{k+1} \\
&= \sum_{k=1}^{\infty}\frac{1}{k!}A^{k-1}t^k \\
\end{aligned}
$$

It's convenient to convert this model to incremental model

$$
\begin{aligned}
\hat{x}_{k+1} &= A_d\hat{x}_k+B_d\hat{u}_k+w_k \\
&= A_d\hat{x}_k + B_d(\hat{u}_{k-1} + \Delta{\hat{u}_k}) + w_k \\
&= A_d\hat{x}_k + B_d\hat{u}_{k-1} + B_d\Delta{\hat{u}_k} + w_k
\end{aligned}
$$

introducing the augmented state variable $\eta$, defined as 

$$
\begin{aligned}
\eta_k = \begin{bmatrix} \hat{x}_{k} \\ \hat{u}_{k-1} \end{bmatrix}
\end{aligned}
$$

the incremental model becomes

$$
\begin{aligned}
\eta_{k+1} = \widetilde{A}_d \eta_k + \widetilde{B}_d \Delta{\hat{u}_k}+\widetilde{w}_k
\end{aligned}
$$

where

$$
\begin{aligned}
\widetilde{A}_d &= \begin{bmatrix} A_d & B_d \\ 0_{n_c \cdot n_s} & I_{n_c \cdot n_c} \end{bmatrix} \\
\widetilde{B}_d &= \begin{bmatrix} B_d \\ I_{n_c \cdot n_c} \end{bmatrix} \\
\widetilde{w}_k &= \begin{bmatrix} w_k \\ 0_{n_c \cdot 1} \end{bmatrix}
\end{aligned}
$$

## Optimal Smoothing Policy

> *For rest of this section, the subscript $k$ is dropped for simplicity!*
{: .prompt-info }

A instantaneous cost for a trajectory smoothing problem could be expressed as

$$
\begin{aligned}
l(x,u,\Delta u) &= \underbrace{(x-x_r)^TQ(x-x_r)}_{\text{state deviation cost}} + \underbrace{(u-u_r)^TR(u-u_r)}_{\text{control deviation cost}} + \underbrace{\Delta u^TS\Delta u}_{\text{control rate cost}} \\
&= [\underbrace{(x-x_l)}_{\hat{x}}-\underbrace{(x_r-x_l)}_{\hat{x}_r}]^TQ[(x-x_l)-(x_r-x_l)] \\
& \qquad + [\underbrace{u-u_l}_{\hat{u}}-\underbrace{u_r-u_l}_{\hat{u}_r}]^TR[(u-u_l)-(u_r-u_l)] \\
& \qquad + (\Delta \hat{u} + \Delta u_l)^TS(\Delta \hat{u} + \Delta u_l) \\
&= (\hat{x}-\hat{x}_r)^TQ(\hat{x}-\hat{x}_r) + (\hat{u}-\hat{u}_r)^TR(\hat{u}-\hat{u}_r)+
(\Delta \hat{u} + \Delta u_l)^TS(\Delta \hat{u} + \Delta u_l) \\
&= \hat{x}^TQ\hat{x}+2(\underbrace{-Q\hat{x}_r}_{q_x})^T\hat{x}+\hat{u}^TR\hat{u}+2(\underbrace{-R\hat{u}_r}_{r_u})^T\hat{u}+\underbrace{\hat{x}_r^TQ\hat{x}_r}_{q_0}+\underbrace{\hat{u}_r^TR\hat{u}_r}_{r_0}+\Delta \hat{u}^TS\Delta \hat{u} + 2(\underbrace{S\Delta u_l}_{s})^T\Delta \hat{u}+\underbrace{\Delta u_l^TS\Delta u_l}_{s_0} \\
&= \begin{bmatrix}\hat{x} \\ \hat{u} \end{bmatrix}^T \underbrace{\begin{bmatrix} Q & 0_{n_s \cdot n_c} \\ 0_{n_c \cdot n_s} & R \end{bmatrix}}_{D_{\eta \eta}} \underbrace{\begin{bmatrix}\hat{x} \\ \hat{u} \end{bmatrix}}_{\eta} + 2 {\underbrace{\begin{bmatrix} q_x \\ r_u \end{bmatrix}}_{d_{\eta}}}^T \begin{bmatrix} \hat{x} \\ \hat{u} \end{bmatrix} +\underbrace{q_0+r_0}_{d_{0_\eta}} + \Delta \hat{u}^TS\Delta \hat{u} + 2{s}^T\Delta \hat{u}+s_0 \\
l(\eta,\Delta \hat{u}) &= \eta ^T D_{\eta \eta} \eta + 2 d_{\eta}^T \eta + d_{0_\eta} + \Delta \hat{u}^TS\Delta \hat{u} + 2{s}^T\Delta \hat{u}+s_0 \\
&= \begin{bmatrix} \eta \\ \Delta \hat{u} \end{bmatrix}^T\underbrace{\begin{bmatrix} D_{\eta \eta} & 0_{(n_s + n_c) \cdot n_c} \\ 0_{n_c \cdot (n_s+n_c)} & S \end{bmatrix}}_{D} \underbrace{\begin{bmatrix} \eta \\ \Delta \hat{u} \end{bmatrix}}_{\xi} + 2{\underbrace{\begin{bmatrix} d_{\eta} \\ s \end{bmatrix}}_{d}}^T\begin{bmatrix} \eta \\ \Delta \hat{u} \end{bmatrix} + \underbrace{d_{0_\eta}+s_0}_{d_0}\\
&= \xi^T D\xi+2d^T\xi+d_0
\end{aligned}
$$

In [lqr post](/posts/Linear-Quadratic-Optimal-Tracking-for-Time-Varying-Nonlinear-System/), we have shown that the q-function and value function has the form

$$
\begin{aligned}
q(\eta,\Delta \hat{u}) &= \xi^TP\xi+2p^T\xi+p_{0} \\
V(\eta) &= \eta^TZ\eta + 2z^T\eta+z_{0}
\end{aligned}
$$

for base case, the terminal cost is defined as

$$
\begin{aligned}
l_f(x_f,u_f) &= (x-x_{rf})^TQ_f(x-x_{rf}) + (u-u_{rf})^TR_f(u-u_{rf}) \\
&= (\hat{x}-\hat{x}_{rf})^TQ_f(\hat{x}-\hat{x}_{rf}) + (\hat{u}-\hat{u}_{rf})^TR_f(\hat{u}-\hat{u}_{rf}) \\
&= \hat{x}^TQ_f\hat{x}+2(\underbrace{-Q_f\hat{x}_{rf}}_{q_{x_f}})^T\hat{x}+\underbrace{\hat{x}_{rf}^TQ_f\hat{x}_{rf}}_{q_{0f}}+\hat{u}^TR_f\hat{u}+2(\underbrace{-R_f\hat{u}_{rf}}_{r_{u_f}})^T\hat{u}+\underbrace{\hat{u}_{rf}^TR_f\hat{u}_{rf}}_{r_{0f}}\\
&= \begin{bmatrix}\hat{x} \\ \hat{u} \end{bmatrix}^T \underbrace{\begin{bmatrix} Q_f & 0_{n_s \cdot n_c} \\ 0_{n_c \cdot n_s} & R_f \end{bmatrix}}_{D_{\eta_f\eta_f}} \underbrace{\begin{bmatrix}\hat{x} \\ \hat{u} \end{bmatrix}}_{\eta} + 2 {\underbrace{\begin{bmatrix} q_{x_f} \\ r_{u_f} \end{bmatrix}}_{d_{\eta_f}}}^T \begin{bmatrix} \hat{x} \\ \hat{u} \end{bmatrix} +\underbrace{q_{0f}+r_{0f}}_{d_{0_{\eta_f}}}\\
q(\eta,\Delta \hat{u}) = l(\eta,\Delta \hat{u}) &= \eta ^T D_{\eta_f \eta_f} \eta + 2 d_{\eta_f}^T \eta + d_{0_{\eta_f}} \\
&= \begin{bmatrix} \eta \\ \Delta \hat{u} \end{bmatrix}^T\underbrace{\begin{bmatrix} D_{\eta_f \eta_f} & 0_{(n_s + n_c) \cdot n_c} \\ 0_{n_c \cdot (n_s+n_c)} & 0_{n_c \cdot n_c} \end{bmatrix}}_{D_f} \underbrace{\begin{bmatrix} \eta \\ \Delta \hat{u} \end{bmatrix}}_{\xi} + 2{\underbrace{\begin{bmatrix} d_{\eta_f} \\ 0_{n_c \cdot 1} \end{bmatrix}}_{d_f}}^T\begin{bmatrix} \eta \\ \Delta \hat{u} \end{bmatrix} + \underbrace{d_{0_{\eta_f}}}_{d_{0f}}\\
&= \xi^T D_f \xi+2d_f^T\xi+d_{0f} \\
&= \xi^TP\xi+2p^T\xi+p_{0}
\end{aligned}
$$

clearly 

$$
\begin{aligned}
P &= D_f \\
p &= d_f \\
p_0 &= d_{0f}
\end{aligned}
$$

assume minimum solution is feasible, we have

$$
\begin{aligned}
q(\eta,\Delta \hat{u}) &= l(\eta,\Delta \hat{u}) + V(\eta_{k+1}) \\
q(\eta,\Delta \hat{u}) &= l(x,u,\Delta u) + V(\widetilde{A}_d \eta + \widetilde{B}_d \Delta{\hat{u}}+\widetilde{w})\\
\xi^TP\xi+2p^T\xi+p_{0} &= \xi^T D\xi+2d^T\xi+d_0 + (\widetilde{A}_d \eta + \widetilde{B}_d \Delta{\hat{u}}+\widetilde{w})^TZ_{k+1}(\widetilde{A}_d \eta + \widetilde{B}_d \Delta{\hat{u}}+\widetilde{w}) + 2z_{k+1}^T(\widetilde{A}_d \eta + \widetilde{B}_d \Delta{\hat{u}}+\widetilde{w})+z_{0_{k+1}}\\
\xi^TP\xi+2p^T\xi+p_{0} &= \xi^T D\xi+2d^T\xi+d_0 + (F_d \xi+\widetilde{w})^TZ_{k+1}(F_d \xi+\widetilde{w}) + 2z_{k+1}^T(F_d \xi+\widetilde{w})+z_{0_{k+1}}\\
& \qquad \text{where $F_d=\begin{bmatrix} \widetilde{A}_d & \widetilde{B}_d \end{bmatrix}$}\\
\xi^TP\xi+2p^T\xi+p_{0} &= \xi^T D\xi+2d^T\xi+d_0 + (\xi^TF_d^TZ_{k+1}+\widetilde{w}^TZ_{k+1})(F_d \xi+\widetilde{w}) + 2(F_d^Tz_{k+1})^T \xi+2z_{k+1}^T\widetilde{w}+z_{0_{k+1}} \\
\xi^TP\xi+2p^T\xi+p_{0} &= \xi^T D\xi+2d^T\xi+d_0 + (\xi^TF_d^TZ_{k+1}+\widetilde{w}^TZ_{k+1})(F_d \xi+\widetilde{w}) + 2(F_d^Tz_{k+1})^T\xi+2z_{k+1}^T\widetilde{w}+z_{0_{k+1}} \\
\xi^TP\xi+2p^T\xi+p_{0} &= \xi^T D\xi+2d^T\xi+d_0 + \xi^TF_d^TZ_{k+1}F_d\xi+2(F_d^TZ_{k+1}\widetilde{w})^T\xi + \widetilde{w}^TZ_{k+1}\widetilde{w} + 2(F_d^Tz_{k+1})^T \xi+2z_{k+1}^T\widetilde{w}+z_{0_{k+1}} \\
\xi^TP\xi+2p^T\xi+p_{0} &= \xi^T (D+F_d^TZ_{k+1}F_d)\xi+2(d+F_d^TZ_{k+1}\widetilde{w}+F_d^Tz_{k+1})^T\xi+ \widetilde{w}^TZ_{k+1}\widetilde{w}+2z_{k+1}^T\widetilde{w}+z_{0_{k+1}}+d_{0} \\
\end{aligned}
$$

Collect quadratic, linear and offset terms

$$
\begin{aligned}
P &= D+F_d^TZ_{k+1}F_d \\
p &= d+F_d^TZ_{k+1}\widetilde{w}+F_d^Tz_{k+1} \\
p_0 &= \widetilde{w}^TZ_{k+1}\widetilde{w}+2z_{k+1}^T\widetilde{w}+z_{0_{k+1}}+d_{0}
\end{aligned}
$$

quadratic coefficient

$$
\begin{aligned}
P &= D+F_d^TZ_{k+1}F_d \\
P &= \begin{bmatrix} D_{\eta \eta} & 0_{(n_s + n_c) \cdot n_c} \\ 0_{n_c \cdot (n_s+n_c)} & S \end{bmatrix} + 
\begin{bmatrix}\widetilde{A}_d^T\\\widetilde{B}_d^T\end{bmatrix}Z_{k+1}\begin{bmatrix}\widetilde{A}_d&\widetilde{B}_d\end{bmatrix}\\
P &= \begin{bmatrix} D_{\eta \eta} & 0_{(n_s + n_c) \cdot n_c} \\ 0_{n_c \cdot (n_s+n_c)} & S \end{bmatrix} + 
\begin{bmatrix} \widetilde{A}_d^TZ_{k+1}\widetilde{A}_d & \widetilde{A}_d^TZ_{k+1}\widetilde{B}_d \\ \widetilde{B}_d^TZ_{k+1}\widetilde{A}_d & \widetilde{B}_d^TZ_{k+1}\widetilde{B}_d \end{bmatrix}\\
P &= \begin{bmatrix} D_{\eta \eta}+\widetilde{A}_d^TZ_{k+1}\widetilde{A}_d & \widetilde{A}_d^TZ_{k+1}\widetilde{B}_d \\ \widetilde{B}_d^TZ_{k+1}\widetilde{A}_d & S+\widetilde{B}_d^TZ_{k+1}\widetilde{B}_d \end{bmatrix}\\
\begin{bmatrix} P_{xx} & P_{xu} \\ P_{xu}^T & P_{uu} \end{bmatrix} &= \begin{bmatrix} D_{\eta \eta}+\widetilde{A}_d^TZ_{k+1}\widetilde{A}_d & \widetilde{A}_d^TZ_{k+1}\widetilde{B}_d \\ \widetilde{B}_d^TZ_{k+1}\widetilde{A}_d & S+\widetilde{B}_d^TZ_{k+1}\widetilde{B}_d \end{bmatrix}
\end{aligned}
$$

linear coefficient

$$
\begin{aligned}
p &= d+F_d^TZ_{k+1}\widetilde{w}+F_d^Tz_{k+1}\\
p &= \begin{bmatrix} d_{\eta} \\ s \end{bmatrix} + \begin{bmatrix}\widetilde{A}_d^T\\\widetilde{B}_d^T\end{bmatrix}Z_{k+1}\widetilde{w}+\begin{bmatrix}\widetilde{A}_d^T\\\widetilde{B}_d^T\end{bmatrix}z_{x_{k+1}}\\
&= \begin{bmatrix} d_{\eta} \\ s \end{bmatrix}+\begin{bmatrix} \widetilde{A}_d^TZ_{k+1}\widetilde{w} \\ \widetilde{B}_d^TZ_{k+1}\widetilde{w} \end{bmatrix} + \begin{bmatrix} \widetilde{A}_d^Tz_{x_{k+1}} \\ \widetilde{B}_d^Tz_{x_{k+1}} \end{bmatrix} \\
& = \begin{bmatrix} d_{\eta}+\widetilde{A}_d^T(Z_{k+1}\widetilde{w}+z_{x_{k+1}})\\
s+\widetilde{B}_d^T(Z_{k+1}\widetilde{w}+z_{x_{k+1}})\end{bmatrix} \\
\begin{bmatrix} p_{x} \\ p_{u} \end{bmatrix}& = \begin{bmatrix} d_{\eta}+\widetilde{A}_d^T(Z_{k+1}\widetilde{w}+z_{x_{k+1}})\\
s+\widetilde{B}_d^T(Z_{k+1}\widetilde{w}+z_{x_{k+1}})\end{bmatrix} 
\end{aligned}
$$

offset term

$$
\begin{aligned}
p_0 &= \widetilde{w}^TZ_{k+1}\widetilde{w}+2z_{k+1}^T\widetilde{w}+z_{0_{k+1}}+d_{0}
\end{aligned}
$$

## Expanding Recursion Equations with Optimal Policy

The optimal policy could be derived by taking partial derivative of q-function

$$
\begin{aligned}
q(\eta,\Delta \hat{u}) &= \begin{bmatrix} \eta \\ \Delta \hat{u} \end{bmatrix}^T\begin{bmatrix} P_{xx} & P_{xu} \\ P_{ux} & P_{uu} \end{bmatrix} \begin{bmatrix} \eta \\ \Delta \hat{u} \end{bmatrix} + 2\begin{bmatrix} p_x \\ p_u\end{bmatrix}^T\begin{bmatrix} \eta \\ \Delta \hat{u} \end{bmatrix}+p_{0} \\
&= \eta^TP_{xx}\eta+2\eta^TP_{xu}\Delta \hat{u}+\Delta \hat{u}^TP_{uu}\Delta \hat{u}+2p_{x}^T\eta+2p_{u}^T\Delta \hat{u}+p_{0}\\
\\
\frac{\partial q(\eta,\Delta \hat{u})}{\partial \Delta \hat{u}} &= 2P_{uu}\Delta \hat{u}^{\ast}+2P_{ux}\eta+2p_{u} = 0\\
0 &= P_{uu}\Delta \hat{u}^{\ast}+P_{ux}\eta+p_{u} \\
\\
\Delta \hat{u}^{\ast} &= -P_{uu}^{-1}P_{ux}\eta-P_{uu}^{-1}p_{u}  \\
&= K\eta+k \\
\\
K &= -P_{uu}^{-1}P_{ux}\\
k &= -P_{uu}^{-1}p_{u}
\end{aligned}
$$

substitute the optimal policy $\Delta \hat{u}^{\ast}$ into q-function to extract value function,with inductive hypothesis,we have

$$
\begin{aligned}
V(\eta) &= q(\eta,\Delta \hat{x}^{\ast}) \\ 
&= \eta^T(P_{xx}+2P_{xu}K+K^TP_{uu}K)\eta + 2(P_{xu}k+K^TP_{uu}k+K^Tp_{u}+p_{x})^T\eta+(k^TP_{uu}k+2p_{u}^Tk+p_{0})\\
&= \eta^T(P_{xx}-P_{xu}P_{uu}^{-1}P_{ux})\eta + 2(p_{x}-P_{xu}P_{uu}^{-1}p_{u})^T\eta+(p_{0}-p_{u}^TP_{uu}^{-1}p_{u})\\
&= \eta^TZ\eta_k+2z^T\eta+z_{0}
\end{aligned}
$$

expanding the recursion equations, we arrive final discrete-time iterative riccati equations

$$
\begin{aligned}
Z &= P_{xx}-P_{xu}P_{uu}^{-1}P_{ux} \\
&= (D_{\eta \eta}+\widetilde{A}_d^TZ_{k+1}\widetilde{A}_d) - (\widetilde{B}_d^TZ_{k+1}\widetilde{A}_d)^T(S+\widetilde{B}_d^TZ_{k+1}\widetilde{B}_d)^{-1}(\widetilde{B}_d^TZ_{k+1}\widetilde{A}_d)
\\
z &= p_{x}-P_{xu}P_{uu}^{-1}p_{u}\\
&= [d_{\eta}+\widetilde{A}_d^T(Z_{k+1}\widetilde{w}+z_{x_{k+1}})]-(\widetilde{B}_d^TZ_{k+1}\widetilde{A}_d)^T(S+\widetilde{B}_d^TZ_{k+1}\widetilde{B}_d)^{-1}[s+\widetilde{B}_d^T(Z_{k+1}\widetilde{w}+z_{x_{k+1}})]
\\
z_{0} &= p_{0}-p_{u}^TP_{uu}^{-1}p_{u}\\
&= [d_{0}+\widetilde{w}^TZ_{k+1}\widetilde{w}+2z_{k+1}^T\widetilde{w}+z_{0_{k+1}}]-[s+\widetilde{B}_d^T(Z_{k+1}\widetilde{w}+z_{x_{k+1}})]^T(S+\widetilde{B}_d^TZ_{k+1}\widetilde{B}_d)^{-1}[s+\widetilde{B}_d^T(Z_{k+1}\widetilde{w}+z_{x_{k+1}})]
\end{aligned}
$$

The optimal policy is

$$
\begin{aligned}
\Delta \hat{u}^{\ast} &= K\eta+k \\
\\
u^{\ast} &= u_{k-1}^{\ast} + \Delta u^{\ast}\\ 
&= u_{k-1}^{\ast} + (\Delta \hat{u}^{\ast}+ \Delta u_l)\\
&= u_{k-1}^{\ast} + \Delta u_l + K\eta+k\\
\\
K &= -P_{uu}^{-1}P_{ux} \\
&= -(S+\widetilde{B}_d^TZ_{k+1}\widetilde{B}_d)^{-1}(\widetilde{B}_d^TZ_{k+1}\widetilde{A}_d) \\
k &= -P_{uu}^{-1}p_{u} \\
&= -(S+\widetilde{B}_d^TZ_{k+1}\widetilde{B}_d)^{-1}[s+\widetilde{B}_d^T(Z_{k+1}\widetilde{w}+z_{x_{k+1}})] \\
\end{aligned}
$$
