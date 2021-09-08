Shader "custom/2sided" { //Hair-Shader-No-BackFace-Culling" {
    Properties{
    //_Color("Main Color", Color) = (1,1,1,1)
    //_SpecColor("Specular Color", Color) = (0.5, 0.5, 0.5, 0)
    //_Shininess("Shininess", Range(0.01, 1)) = 0.078125
    //_MainTex("Base (RGB) TransGloss (A)", 2D) = "white" {}
    //_BumpMap("Normalmap", 2D) = "bump" {}
    //_Cutoff("Alpha cutoff", Range(0,1)) = 0.5
    }

        SubShader{
        Cull Off
        Tags {"Queue" = "AlphaTest" "IgnoreProjector" = "True"  "RenderType" = "TransparentCutout"}
        LOD 200 //400

        CGPROGRAM
        #pragma surface surf Lambert vertex:vert
        #pragma exclude_renderers flash

        //sampler2D _MainTex;
        //sampler2D _BumpMap;
        //fixed4 _Color;
        //half _Shininess;

        struct Input {
			float4 vertColor;
        //float2 uv_MainTex;
        //float2 uv_BumpMap;
        };
		
		void vert(inout appdata_full v, out Input o) {
			UNITY_INITIALIZE_OUTPUT(Input, o);
			o.vertColor = v.color;
		}
		
        void surf(Input IN, inout SurfaceOutput o) {
			o.Albedo = IN.vertColor.rgb;
        //fixed4 tex = tex2D(_MainTex, IN.uv_MainTex);
        //o.Albedo = tex.rgb * _Color.rgb;
        //o.Gloss = tex.a;
        //o.Alpha = tex.a * _Color.a;
        //o.Specular = IN.vertColor.rgb;
        //o.Normal = UnpackNormal(tex2D(_BumpMap, IN.uv_BumpMap));
        }
        ENDCG
    }

        //FallBack "Transparent/Cutout/VertexLit"
}