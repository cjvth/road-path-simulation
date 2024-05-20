[bbox:54.8, 82.6, 55.2, 83.2];
(
  (
    (way [highway~"^(motorway|motorway_link|trunk|trunk_link\
            |primary|primary_link|secondary|secondary_link\
            |tertiary|tertiary_link|unclassified\
            |residential|residential_link|unclassified)$"]
      ({{bbox}});
     >;
   );
    rel[type=restriction](bw);
    <<;
  );
  //>;
)->.roads;

(
  way[building]
  ({{bbox}});
  >>;
)->.buildings;

(
  .roads;
  .buildings;
);

out body;

####

Мост

выгрузка с оверпасс

 construction


ручное добавление

  <way id="767676767">
    <nd ref="280879734"/>
    <nd ref="701712113"/>
    <tag k="highway" v="trunk"/>
    <tag k="oneway" v="yes"/>
    <tag k="ADITIONAL" v="TRUE"/>
  </way>
 <way id="6767676767">
    <nd ref="701712113"/>
    <nd ref="277278438"/>
    <tag k="highway" v="trunk"/>
    <tag k="oneway" v="yes"/>
    <tag k="lanes" v="3"/>
    <tag k="ADITIONAL" v="TRUE"/>
  </way>
