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
  final parameter Modelica.SIunits.Volume V=L*pi*(D/2)^2;

  parameter Modelica.SIunits.MassFlowRate m_flow_nominal;
  parameter Modelica.SIunits.PressureDifference dp_nominal=0;

  parameter Modelica.SIunits.Density rho = 1000 "Mass density of fluid";
  parameter Modelica.SIunits.SpecificHeatCapacity cp=4187
    "Specific heat of fluid";

  parameter Boolean dynamicBalance = true
    "Set to true to use a dynamic balance, which often leads to smaller systems of equations"
    annotation (Evaluate=true, Dialog(tab="Dynamics", group="Equations"));
  parameter Modelica.SIunits.ThermalConductivity k=0.026 "Heat conductivity";
  parameter Real S=2*pi/Modelica.Math.log((D+h)/D) "Shape factor";

  final parameter Modelica.SIunits.ThermalConductivity r=1/(k*S);
  final parameter Real c=rho*pi*(D/2)^2*cp;

  //Variables
  Real u;

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
    annotation (Placement(transformation(extent={{-26,18},{-6,38}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=u)
    annotation (Placement(transformation(extent={{-66,18},{-46,38}})));
  BaseClasses.ExponentialDecay tempDecay(C=c, R=r)
    annotation (Placement(transformation(extent={{20,14},{40,34}})));
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

  connect(port_a, plugFlowPipe.port_a) annotation (Line(
      points={{-100,0},{-60,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pDETime.u, realExpression.y) annotation (Line(
      points={{-28,28},{-45,28}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(plugFlowPipe.port_b, senTem.port_a) annotation (Line(
      points={{-40,0},{-10,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(senTem.T, tempDecay.TIn) annotation (Line(
      points={{0,11},{0,20},{18,20}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pDETime.tau, tempDecay.tDelay) annotation (Line(
      points={{-5,28},{18,28}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(TBoundary, tempDecay.TBou) annotation (Line(
      points={{0,110},{0,66},{30,66},{30,36}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(tempDecay.TOut, idealHeater.TSet) annotation (Line(
      points={{41,24},{50,24},{50,6},{58,6}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(senTem.port_b, idealHeater.port_a) annotation (Line(
      points={{10,0},{60,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater.port_b, port_b) annotation (Line(
      points={{80,0},{100,0}},
      color={0,127,255},
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