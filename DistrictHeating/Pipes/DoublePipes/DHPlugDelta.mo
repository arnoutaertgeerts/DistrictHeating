within DistrictHeating.Pipes.DoublePipes;
model DHPlugDelta "Delta circuit DH pipe with plug flow"

  //Extensions
  extends BaseClasses.PartialDistrictHeatingPipe(
    final energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState);

  //Components
  PlugFlowHeatLossTwinPipe plugFlowHeatLossTwinPipe(
    redeclare package Medium = Medium,
    L=L,
    ha=ha,
    hs=hs,
    m_flow_nominal=m_flow_nominal,
    k=lambdaI,
    D=Di,
    computeFlowResistance=computeFlowResistance,
    from_dp=from_dp,
    dp_nominal=dp_nominal,
    linearizeFlowResistance=linearizeFlowResistance,
    deltaM=deltaM,
    allowFlowReversal=allowFlowReversal)
    annotation (Placement(transformation(extent={{-30,-30},{30,30}})));
equation
  Q1 = plugFlowHeatLossTwinPipe.Q_1*L;
  Q2 = plugFlowHeatLossTwinPipe.Q_2*L;

  connect(port_a1, plugFlowHeatLossTwinPipe.port_a1) annotation (Line(
      points={{-100,60},{-60,60},{-60,18},{-30,18}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plugFlowHeatLossTwinPipe.port_b1, port_b1) annotation (Line(
      points={{30,18},{60,18},{60,60},{100,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plugFlowHeatLossTwinPipe.port_b2, port_b2) annotation (Line(
      points={{-30,-18},{-60,-18},{-60,-60},{-100,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(plugFlowHeatLossTwinPipe.port_a2, port_a2) annotation (Line(
      points={{30,-18},{60,-18},{60,-60},{100,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(Tg, plugFlowHeatLossTwinPipe.TBoundary) annotation (Line(
      points={{0,-142},{0,-44},{50,-44},{50,54},{0,54},{0,30}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Icon(coordinateSystem(extent={{-100,-140},{100,140}}), graphics={
                                 Text(
          extent={{-151,147},{149,107}},
          lineColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,127,255},
          textString="%name"),
        Rectangle(
          extent={{-28,90},{28,30}},
          lineColor={255,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={255,0,0}),
        Rectangle(
          extent={{-28,-30},{28,-90}},
          lineColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder),
        Polygon(
          points={{0,20},{-20,-20},{20,-20},{0,20}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          smooth=Smooth.None,
          fillColor={0,127,0})}),            Diagram(coordinateSystem(extent={{-100,
            -140},{100,140}}, preserveAspectRatio=false), graphics));
end DHPlugDelta;
