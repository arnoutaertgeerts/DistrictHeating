within DistrictHeating.Pipes.BaseClasses;
model TwinExponentialDecay
  "Calculates decay in temperature for given inlet, delay and boundary conditions"

  parameter Real C;
  parameter Real Ra;
  parameter Real Rs;

  final parameter Real tau=C*sqrt(Ra*Rs);

  Modelica.Blocks.Interfaces.RealInput T1 "Inlet temperature at time t-delay"
    annotation (Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-20})));
  Modelica.Blocks.Interfaces.RealInput td
    "Delay time for current package of fluid"
    annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
  Modelica.Blocks.Interfaces.RealInput Tb
    "Boundary temperature - Fluid would cool down to this temperature if it were to stay long enough in pipe"
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,40})));
  Modelica.Blocks.Interfaces.RealOutput T1Out "Oulet temperature"
    annotation (Placement(transformation(extent={{100,30},{120,50}})));

  Modelica.Blocks.Interfaces.RealInput T2 "Inlet temperature at time t-delay"
    annotation (Placement(transformation(extent={{20,-20},{-20,20}},
        rotation=180,
        origin={-120,-60})));
  Modelica.Blocks.Interfaces.RealOutput T2Out "Oulet temperature"
    annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
equation
  T1Out = Tb
    + (T1-Tb)*Modelica.Math.exp(-td/tau)
    + (Rs-Ra)/(Ra+Rs)*((Tb-T2)*Modelica.Math.exp(-2*td/tau)+T2-Tb);

  T2Out = Tb
    + (T2-Tb)*Modelica.Math.exp(-td/tau)
    + (Rs-Ra)/(Ra+Rs)*((Tb-T1)*Modelica.Math.exp(-2*td/tau)+T1-Tb);

    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(
          preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
                                Rectangle(
        extent={{-100,-100},{100,100}},
        lineColor={0,0,127},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid),
        Text(
          extent={{-60,140},{60,100}},
          lineColor={0,0,255},
          textString="%name"),
        Line(
          points={{-80,82},{-80,-78}},
          color={0,0,0},
          smooth=Smooth.None),
        Line(
          points={{-80,-78},{80,-78}},
          color={0,0,0},
          smooth=Smooth.None),
        Line(
          points={{-90,62},{-80,82},{-70,62}},
          color={0,0,0},
          smooth=Smooth.None),
        Line(
          points={{-10,-10},{0,10},{10,-10}},
          color={0,0,0},
          smooth=Smooth.None,
          origin={70,-78},
          rotation=270),
        Line(
          points={{-80,60},{-34,-56},{52,-60}},
          color={255,128,0},
          smooth=Smooth.Bezier),
        Line(
          points={{66,60},{20,-56},{-66,-60}},
          color={255,128,0},
          smooth=Smooth.Bezier,
          origin={-14,2},
          rotation=360)}));
end TwinExponentialDecay;
