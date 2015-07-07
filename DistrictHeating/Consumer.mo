within DistrictHeating;
model Consumer "Emulates a consumer with a given heat demand"

  parameter Modelica.SIunits.Temperature TSupply = 90+273.15
    "Desired temperature of supply side";
  parameter Modelica.SIunits.Temperature TReturn = 70+273.15
    "Desired temperature of return side";
  parameter Modelica.SIunits.MassFlowRate dm_nom = 12.5
    "Calculated peak mass flow";
  parameter Modelica.SIunits.SpecificHeatCapacity  cp=4195;
  replaceable package Medium =
      Modelica.Media.Interfaces.PartialMedium "Medium in the component"
      annotation (choicesAllMatching = true);
  parameter Real eff = 0.9
    "Efficiency of heating system in order to calculate final demand";
  parameter String filePath = "" "Location of demand data"
    annotation (Dialog(group="Geometry"), choices(
    choice="M:/Documents/Holdub/MatData/CW.mat" "CW",
    choice="M:/Documents/Holdub/MatData/Geo.mat" "Geo",
    choice="M:/Documents/Holdub/MatData/KanNatuurkunde.mat" "KanNatuurkunde",
    choice="M:/Documents/Holdub/MatData/NavNatuurkunde.mat" "NavNatuurkunde",
    choice="M:/Documents/Holdub/MatData/Scheikunde.mat" "Scheikunde",
    choice="M:/Documents/Holdub/MatData/Wiskunde.mat" "Wiskunde"));

  Annex60.Fluid.Movers.FlowControlled_m_flow fan(redeclare package Medium =
        Medium, m_flow_nominal=dm_nom)           annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-40,-26})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a1(redeclare package Medium =
        Medium)
    "Fluid connector a (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{-70,-110},{-50,-90}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn(T_start=TSupply, redeclare package
      Medium = Medium,
    m_flow_nominal=dm_nom,
    tau=0.001)              annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-40,-62})));
  Modelica.Blocks.Sources.CombiTimeTable demand(
    tableOnFile=true,
    fileName="M:/Documents/Holdub/MatData/",
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    extrapolation=Modelica.Blocks.Types.Extrapolation.NoExtrapolation,
    tableName="TimeEnergy",
    columns={2})
    annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b1(redeclare package Medium =
        Medium)
    "Fluid connector b (positive design flow direction is from port_a to port_b)"
    annotation (Placement(transformation(extent={{50,-110},{70,-90}})));
  IDEAS.Fluid.HeatExchangers.HeaterCooler_u hea1(redeclare package Medium =
               Medium,
    m_flow_nominal=dm_nom,
    dp_nominal=0,
    Q_flow_nominal=cp*(TSupply - TReturn)*dm_nom)
    annotation (Placement(transformation(extent={{0,0},{20,20}})));
  Modelica.Blocks.Math.Gain gain(k=-eff) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-18,48})));
  Modelica.Blocks.Math.Gain gain1(k=1/(cp*(TSupply - TReturn)))                            annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-70,-6})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOut(                redeclare package
      Medium = Medium, T_start=TReturn,
    tau=0.001,
    m_flow_nominal=dm_nom)  annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={40,-60})));
equation
  connect(fan.port_a, TIn.port_b) annotation (Line(
      points={{-40,-36},{-40,-52}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(port_a1, TIn.port_a) annotation (Line(
      points={{-60,-100},{-60,-86},{-40,-86},{-40,-72}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fan.port_b, hea1.port_a) annotation (Line(
      points={{-40,-16},{-40,10},{0,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(demand.y[1], gain.u) annotation (Line(
      points={{-59,70},{-18,70},{-18,60}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(gain.y, hea1.u) annotation (Line(
      points={{-18,37},{-18,24},{-10,24},{-10,16},{-2,16}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(gain.y, gain1.u) annotation (Line(
      points={{-18,37},{-18,24},{-70,24},{-70,6}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(gain1.y, fan.m_flow_in) annotation (Line(
      points={{-70,-17},{-70,-26.2},{-52,-26.2}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(hea1.port_b, TOut.port_a) annotation (Line(
      points={{20,10},{40,10},{40,-50}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TOut.port_b, port_b1) annotation (Line(
      points={{40,-70},{40,-100},{60,-100}},
      color={0,127,255},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), Icon(coordinateSystem(
          preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-60,-100},{-60,-6}},
          color={255,0,0},
          smooth=Smooth.None,
          thickness=0.5),
        Rectangle(
          extent={{-74,40},{74,-6}},
          lineColor={255,0,0},
          lineThickness=0.5,
          fillPattern=FillPattern.VerticalCylinder,
          fillColor={255,0,0}),
        Line(
          points={{60,-100},{60,-6}},
          color={0,0,255},
          thickness=0.5,
          smooth=Smooth.None),
        Polygon(
          points={{-6,9},{0,-9},{6,9},{-6,9}},
          lineColor={255,0,0},
          lineThickness=0.5,
          fillPattern=FillPattern.VerticalCylinder,
          smooth=Smooth.None,
          fillColor={255,0,0},
          origin={-60,-15},
          rotation=180),
        Polygon(
          points={{-6,9},{0,-9},{6,9},{-6,9}},
          lineColor={0,0,255},
          lineThickness=0.5,
          fillPattern=FillPattern.VerticalCylinder,
          smooth=Smooth.None,
          fillColor={0,0,255},
          origin={60,-81},
          rotation=360),
        Text(
          extent={{-84,150},{78,106}},
          lineColor={0,0,255},
          lineThickness=0.5,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid,
          textString="%name")}));
end Consumer;
