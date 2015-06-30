within DistrictHeating.Pipes.BaseClasses;
model ExponentialDecay
  "Calculates decay in temperature for given inlet, delay and boundary conditions"

  parameter Real C;
  parameter Real R;

  Modelica.Blocks.Interfaces.RealInput TIn "Inlet temperature at time t-delay"
    annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
  Modelica.Blocks.Interfaces.RealInput tDelay
    "Delay time for current package of fluid"
    annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
  Modelica.Blocks.Interfaces.RealInput TBou
    "Boundary temperature - Fluid would cool down to this temperature if it were to stay long enough in pipe"
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,120})));
  Modelica.Blocks.Interfaces.RealOutput TOut "Oulet temperature"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));

equation
  TOut = TBou + (TIn - TBou)*Modelica.Math.exp(-tDelay/(C*R));

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
        Text(
          extent={{-50,86},{86,16}},
          lineColor={0,0,255},
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid,
          textString="exp(-t/tau)")}));
end ExponentialDecay;
