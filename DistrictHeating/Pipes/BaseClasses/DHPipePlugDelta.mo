within DistrictHeating.Pipes.BaseClasses;
model DHPipePlugDelta "Delta circuit DH pipe with plug flow"

  //Extensions
  extends PartialDistrictHeatingPipe;

  //Components
  PlugFlowHeatLossTwinPipe plugFlowHeatLossTwinPipe(
    redeclare package Medium = Medium,
    L=L,
    ha=ha,
    hs=hs,
    m_flow_nominal=m_flow_nominal,
    k=lambdaI,
    dp_nominal=dp_nominal,
    D=Di)
    annotation (Placement(transformation(extent={{-30,-30},{30,30}})));
equation

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
        Polygon(
          points={{30,22},{60,12},{30,0},{30,22}},
          smooth=Smooth.None,
          fillColor={255,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Ellipse(
          extent={{30,-92},{-30,-32}},
          lineColor={0,0,255},
          fillColor={0,0,255},
          fillPattern=FillPattern.Sphere),
        Polygon(
          points={{30,18},{52,12},{30,4},{30,18}},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(
          points={{-60,12},{56,12}},
          color={255,0,0},
          smooth=Smooth.None),
        Polygon(
          points={{-28,0},{-58,-10},{-28,-22},{-28,0}},
          smooth=Smooth.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Polygon(
          points={{-28,-4},{-50,-10},{-28,-18},{-28,-4}},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.HorizontalCylinder,
          pattern=LinePattern.None,
          lineColor={0,0,0}),
        Line(
          points={{62,-10},{-50,-10}},
          color={0,0,255},
          smooth=Smooth.None),
        Ellipse(
          extent={{30,30},{-30,90}},
          lineColor={255,0,0},
          fillColor={255,0,0},
          fillPattern=FillPattern.Sphere)}), Diagram(coordinateSystem(extent={{-100,
            -140},{100,140}}, preserveAspectRatio=false), graphics));
end DHPipePlugDelta;
