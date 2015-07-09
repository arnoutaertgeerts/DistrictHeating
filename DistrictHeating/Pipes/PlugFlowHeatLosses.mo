within DistrictHeating.Pipes;
model PlugFlowHeatLosses
  "Pipe model with a temperature plug flow, pressure losses and heat exchange to the environment"

  //Extensions
  extends IDEAS.Fluid.Interfaces.PartialTwoPortInterface;
  extends IDEAS.Fluid.Interfaces.LumpedVolumeDeclarations;

  //Parameters
  parameter Modelica.SIunits.Length L;
  parameter Modelica.SIunits.Length D;
  parameter Modelica.SIunits.Length h=0.02 "Insulation thickness";

  final constant Real pi = Modelica.Constants.pi;
  final parameter Modelica.SIunits.Area A=pi*(D/2)^2;
  final parameter Modelica.SIunits.Volume V=L*A;

  parameter Modelica.SIunits.MassFlowRate m_flow_nominal;
  parameter Modelica.SIunits.PressureDifference dp_nominal=0;

  parameter Modelica.SIunits.Density rho = 1000 "Mass density of fluid";
  parameter Modelica.SIunits.SpecificHeatCapacity cp=4187
    "Specific heat of fluid";

  parameter Boolean dynamicBalance = true
    "Set to true to use a dynamic balance, which often leads to smaller systems of equations"
    annotation (Evaluate=true, Dialog(tab="Dynamics", group="Equations"));
  parameter Modelica.SIunits.ThermalConductivity lambdaI=0.026
    "Heat conductivity";

  parameter Modelica.SIunits.ThermalConductivity R=1/(lambdaI*2*pi/Modelica.Math.log((D/2+h)/(D/2)));
  final parameter Real C=rho*pi*(D/2)^2*cp;

  //Variables
  Real u;
  Modelica.SIunits.Power Q_Losses;

  //Components
  DistrictHeating.Pipes.PlugFlowPipe plugFlowPipe(
    pipeLength=L,
    pipeDiameter=D,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=dp_nominal,
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));

  Modelica.Blocks.Interfaces.RealInput TBoundary annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,110}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={2,50})));
  TimeDelays.PDETime pDETime
    annotation (Placement(transformation(extent={{-28,24},{-8,44}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=u)
    annotation (Placement(transformation(extent={{-58,24},{-38,44}})));
  BaseClasses.ExponentialDecay tempDecay(C=C, R=R)
    annotation (Placement(transformation(extent={{20,20},{40,40}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort senTem(m_flow_nominal=m_flow_nominal,
      redeclare package Medium = Medium,
    tau=0)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater(
    m_flow_nominal=m_flow_nominal,
    redeclare package Medium = Medium,
    dp_nominal=0)
    annotation (Placement(transformation(extent={{60,-10},{80,10}})));
equation
  //Normalized speed of the fluid [1/s]
  u = port_a.m_flow/(rho*V);
  Q_Losses = -idealHeater.Q_flow/L;

  connect(port_a, plugFlowPipe.port_a) annotation (Line(
      points={{-100,0},{-60,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pDETime.u, realExpression.y) annotation (Line(
      points={{-30,34},{-37,34}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(plugFlowPipe.port_b, senTem.port_a) annotation (Line(
      points={{-40,0},{-10,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(senTem.port_b, idealHeater.port_a) annotation (Line(
      points={{10,0},{60,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater.port_b, port_b) annotation (Line(
      points={{80,0},{100,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(senTem.T, tempDecay.TIn) annotation (Line(
      points={{0,11},{0,26},{18,26}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pDETime.tau, tempDecay.td) annotation (Line(
      points={{-7,34},{18,34}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(tempDecay.TOut, idealHeater.TSet) annotation (Line(
      points={{41,30},{50,30},{50,6},{58,6}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TBoundary, tempDecay.Tb) annotation (Line(
      points={{0,110},{0,60},{30,60},{30,42}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {100,100}}),
                   graphics={
        Polygon(
          points={{20,-70},{60,-85},{20,-100},{20,-70}},
          lineColor={0,128,255},
          smooth=Smooth.None,
          fillColor={0,128,255},
          fillPattern=FillPattern.Solid,
          visible=showDesignFlowDirection),
        Polygon(
          points={{20,-75},{50,-85},{20,-95},{20,-75}},
          lineColor={255,255,255},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          visible=allowFlowReversal),
        Polygon(
          points={{20,-75},{50,-85},{20,-95},{20,-75}},
          lineColor={255,255,255},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          visible=allowFlowReversal),
        Rectangle(
          extent={{-100,40},{100,-40}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={192,192,192}),
        Rectangle(
          extent={{-100,30},{100,-30}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,127,255}),
        Rectangle(
          extent={{-26,30},{30,-30}},
          lineColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder),
        Line(
          points={{55,-85},{-60,-85}},
          color={0,128,255},
          smooth=Smooth.None,
          visible=showDesignFlowDirection),
        Rectangle(
          extent={{-100,50},{100,40}},
          lineColor={175,175,175},
          fillColor={255,255,255},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-100,-40},{100,-50}},
          lineColor={175,175,175},
          fillColor={255,255,255},
          fillPattern=FillPattern.Backward)}),
                           Diagram(coordinateSystem(preserveAspectRatio=false,
          extent={{-100,-100},{100,100}}), graphics));
end PlugFlowHeatLosses;
