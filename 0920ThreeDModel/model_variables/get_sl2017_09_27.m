function sl = get_sl2017_09_27(in1,in2)
%GET_SL2017_09_27
%    SL = GET_SL2017_09_27(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 5.10.
%    27-Sep-2017 17:07:40

phif = in1(4,:);
phig = in1(9,:);
phit = in1(7,:);
psif = in1(6,:);
psig = in1(11,:);
thetaa = in1(12,:);
thetaf = in1(5,:);
thetag = in1(10,:);
thetat = in1(8,:);
t2 = phif-phig;
t3 = phif-phit;
t4 = cos(t3);
t5 = thetaf-thetat;
t6 = pi.*(1.0./2.0);
t7 = t6+thetag-thetat;
t8 = psif-psig;
t9 = cos(t2);
t10 = sin(t3);
t11 = cos(t5);
t12 = sin(t2);
t13 = thetaa-thetag;
t14 = sin(t7);
t15 = t10.*t12;
t37 = t4.*t9.*t11;
t16 = t15-t37;
t17 = sin(t5);
t18 = cos(t7);
t19 = cos(phif);
t20 = sin(psif);
t21 = cos(psif);
t22 = sin(phif);
t23 = -t6+thetaf;
t24 = cos(t23);
t25 = cos(t8);
t26 = sin(t8);
t27 = t19.*t20;
t28 = t27-t21.*t22.*t24;
t29 = t17.*4.0;
t30 = sin(t13);
t31 = cos(t13);
t32 = t11.*t14;
t62 = t9.*t17.*t18;
t33 = t32-t62;
t34 = t20.*t22;
t35 = t19.*t21.*t24;
t36 = t34+t35;
t38 = t16.*t18;
t53 = t4.*t14.*t17;
t39 = t38-t53;
t40 = t9.*t10;
t41 = t4.*t11.*t12;
t42 = t40+t41;
t43 = sin(t23);
t44 = t4.*t12;
t45 = t9.*t10.*t11;
t46 = t44+t45;
t47 = t18.*t46;
t48 = t10.*t14.*t17;
t49 = t47+t48;
t50 = t4.*t9;
t80 = t10.*t11.*t12;
t51 = t50-t80;
t52 = t10.*t11.*4.0;
t54 = t25.*t39;
t94 = t26.*t42;
t55 = t54-t94;
t56 = t31.*t55.*(2.2e1./1.25e2);
t57 = t4.*t11.*4.0;
t58 = t14.*t16;
t59 = t4.*t17.*t18;
t60 = t58+t59;
t95 = t30.*t60.*(2.2e1./1.25e2);
t61 = t56+t57-t95;
t63 = t25.*t33.*(2.13e2./1.0e3);
t96 = t12.*t17.*t26.*(2.13e2./1.0e3);
t64 = t29+t63-t96;
t65 = t19.*t21;
t66 = t20.*t22.*t24;
t67 = t65+t66;
t68 = t11.*t18;
t69 = t9.*t14.*t17;
t70 = t68+t69;
t71 = t30.*t70.*(2.2e1./1.25e2);
t72 = t25.*t33;
t97 = t12.*t17.*t26;
t73 = t72-t97;
t74 = t31.*t73.*(2.2e1./1.25e2);
t75 = t29+t71+t74;
t76 = t21.*t22;
t77 = t76-t19.*t20.*t24;
t78 = t25.*t39.*(2.13e2./1.0e3);
t93 = t26.*t42.*(2.13e2./1.0e3);
t79 = t57+t78-t93;
t81 = t26.*t51.*(2.13e2./1.0e3);
t89 = t25.*t49.*(2.13e2./1.0e3);
t82 = t52+t81-t89;
t83 = t25.*t49;
t90 = t26.*t51;
t84 = t83-t90;
t85 = t14.*t46;
t92 = t10.*t17.*t18;
t86 = t85-t92;
t87 = t30.*t86.*(2.2e1./1.25e2);
t91 = t31.*t84.*(2.2e1./1.25e2);
t88 = t52+t87-t91;
sl = [t28.*t64-t36.*t61-t28.*t75+t36.*t79+t21.*t43.*t82-t21.*t43.*t88;-t64.*t67+t61.*t77+t67.*t75-t77.*t79+t20.*t43.*t82-t20.*t43.*t88;t24.*t82-t24.*t88+t19.*t43.*t61+t22.*t43.*t64-t22.*t43.*t75-t19.*t43.*t79];
