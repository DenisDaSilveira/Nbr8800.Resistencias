using System;
using System.Collections.Generic;

namespace Nbr8800.Resistencias
{
    public enum TipoSecao
    {
        IouHDuplaSimetria,
        IouHMonossimetrica,
        U,
        T,
        DuasCantoneirasEmT,
        Cantoneira,
        TubularRetangular,
        Caixao,
        TubularCircular,
        SolidaRetangular,
        SolidaCircular,
        GenericaDuplaSimetria,
        GenericaMonossimetrica,
        GenericaAssimetrica
    }

    public enum FabricacaoPerfil { Laminado, Soldado, Formado }
    public enum EixoFlexao { X, Y }
    public enum ClasseSecao { Compacta, Semicompacta, Esbelta }
    public enum ClasseElemento { AA, AL, ParedeTubularRetangular }
    public enum GrupoElementoComprimido { Grupo1, Grupo2, Grupo3, Grupo4, Grupo5, Grupo6 }
    public enum EstadoLimite
    {
        EscoamentoSecaoBruta,
        RupturaSecaoLiquida,
        FlexaoX,
        FlexaoY,
        Torcao,
        FlexoTorcao,
        FlambagemLocal,
        FLM,
        FLA,
        FLT,
        PlastificacaoCisalhamento,
        FlambagemCisalhamentoTorcao,
        Empenamento,
        EscoamentoPorTensoesCombinadas,
        RotacaoTorcional
    }

    /// <summary>Unidades coerentes. Recomendado: N e mm, com tensões em MPa.</summary>
    public sealed class MaterialAco
    {
        public double Fy { get; set; }
        public double Fu { get; set; }
        public double E { get; set; } = 200000.0;
        public double G { get; set; } = 77000.0;
        public double GammaA1 { get; set; } = 1.10;
        public double GammaA2 { get; set; } = 1.35;
    }

    public sealed class ElementoComprimido
    {
        public string Nome { get; set; }
        public ClasseElemento Classe { get; set; }
        public GrupoElementoComprimido Grupo { get; set; }
        public double B { get; set; }
        public double T { get; set; }
        public int Quantidade { get; set; } = 1;
        /// <summary>kc da nota da Tabela 4. Se zero, é obtido de H/Tw.</summary>
        public double Kc { get; set; }
        public double H { get; set; }
        public double Tw { get; set; }
    }

    public sealed class SecaoMetalica
    {
        public TipoSecao Tipo { get; set; }
        public FabricacaoPerfil Fabricacao { get; set; }
        public double Ag { get; set; }
        public double An { get; set; }
        public double Ix { get; set; }
        public double Iy { get; set; }
        public double J { get; set; }
        public double Cw { get; set; }
        public double X0 { get; set; }
        public double Y0 { get; set; }
        public double Wx { get; set; }
        public double Wy { get; set; }
        public double Zx { get; set; }
        public double Zy { get; set; }
        public double D { get; set; }
        public double Bf { get; set; }
        public double Tf { get; set; }
        public double H { get; set; }
        public double Tw { get; set; }
        public double Diametro { get; set; }
        public double EspessuraTubo { get; set; }
        public IList<ElementoComprimido> ElementosComprimidos { get; } = new List<ElementoComprimido>();

        public double Rx => Math.Sqrt(Ix / Ag);
        public double Ry => Math.Sqrt(Iy / Ag);
        public double R0Quadrado => Rx * Rx + Ry * Ry + X0 * X0 + Y0 * Y0;

        public double AreaCisalhamentoX { get; set; }

        public double AreaCisalhamentoY { get; set; }
        public double EspessuraMaxima { get; set; }
        public double OmegaMax { get; set; }
    }

    public sealed class ComprimentosFlambagem
    {
        public double Lx { get; set; }
        public double Ly { get; set; }
        public double Lz { get; set; }
    }

    public sealed class ResultadoResistencia
    {
        public double Valor { get; set; }
        public EstadoLimite Governante { get; set; }
        public IDictionary<string, double> Valores { get; } = new Dictionary<string, double>();
        public IList<string> Memoria { get; } = new List<string>();
        public IList<string> Avisos { get; } = new List<string>();
    }

    public sealed class ResultadoClassificacao
    {
        public ClasseSecao Classe { get; set; }
        public IList<string> Memoria { get; } = new List<string>();
    }

    public sealed class ParametrosFlexao
    {
        public EixoFlexao Eixo { get; set; }
        public double Lb { get; set; }
        public double Cb { get; set; } = 1.0;
        /// <summary>Mcr alternativo obtido por análise elástica. Zero usa fórmula implementada quando disponível.</summary>
        public double McrAlternativo { get; set; }
        public double Wef { get; set; }
    }

    public sealed class ParametrosCortante
    {
        public EixoFlexao EixoFlexao { get; set; }
        public double EspacamentoEnrijecedores { get; set; }
        public double Lv { get; set; }
    }

    public class ParametrosTorcao
    {
        /// <summary>
        /// Momento torsor solicitante de cálculo.
        /// </summary>
        public double TSd { get; set; }

        /// <summary>
        /// Parcela do torsor resistida por Saint-Venant.
        /// </summary>
        public double TSaintVenantSd { get; set; }

        /// <summary>
        /// Bimomento solicitante de cálculo.
        /// </summary>
        public double BSd { get; set; }

        /// <summary>
        /// Tensão de cisalhamento máxima de empenamento,
        /// obtida pela análise da barra.
        /// </summary>
        public double TauEmpenamentoSd { get; set; }

        /// <summary>
        /// Tensão normal já existente por força axial e flexão.
        /// Informar com sinal.
        /// </summary>
        public double SigmaNormalSd { get; set; }

        /// <summary>
        /// Tensão de cisalhamento já existente por força cortante.
        /// </summary>
        public double TauCortanteSd { get; set; }

        /// <summary>
        /// Máximo valor absoluto da coordenada setorial da seção.
        /// </summary>
        public double OmegaMax { get; set; }

        /// <summary>
        /// Comprimento da barra analisada.
        /// </summary>
        public double Comprimento { get; set; }

        /// <summary>
        /// Rotação torsional máxima obtida na análise.
        /// </summary>
        public double RotacaoMaxRad { get; set; }

        /// <summary>
        /// Limite de rotação definido pelo projetista.
        /// </summary>
        public double RotacaoLimiteRad { get; set; }

        /// <summary>
        /// Indica que o empenamento nas extremidades está livre.
        /// </summary>
        public bool EmpenamentoLivre { get; set; }

        /// <summary>
        /// Indica que foi realizada análise torsional não uniforme.
        /// </summary>
        public bool AnaliseEmpenamentoRealizada { get; set; }
    }

    public sealed class CondicoesTorsionaisViga
    {
        public RestricaoExtremidadeTorcional Inicial { get; set; }
            = new RestricaoExtremidadeTorcional();

        public RestricaoExtremidadeTorcional Final { get; set; }
            = new RestricaoExtremidadeTorcional();

        /// <summary>
        /// Rigidez rotacional distribuída fornecida pela laje,
        /// travamentos ou outros elementos.
        /// </summary>
        public double KThetaDistribuido { get; set; }
    }

    public enum CondicaoEmpenamentoExtremidade
    {
        Livre,

        /// <summary>
        /// Derivada torsional impedida:
        /// theta' = 0.
        /// </summary>
        Impedido,

        /// <summary>
        /// Restrição representada por mola de empenamento.
        /// </summary>
        Elastico
    }

    public enum CondicaoRotacaoTorcional
    {
        Livre,

        /// <summary>
        /// Rotação torsional impedida:
        /// theta = 0.
        /// </summary>
        Impedida,

        Elastico
    }

    public sealed class RestricaoExtremidadeTorcional
    {
        public CondicaoRotacaoTorcional Rotacao { get; set; }

        public CondicaoEmpenamentoExtremidade Empenamento { get; set; }

        /// <summary>
        /// Rigidez de mola à rotação torsional.
        /// Usada somente quando Rotacao = Elastico.
        /// </summary>
        public double KTheta { get; set; }

        /// <summary>
        /// Rigidez de mola de empenamento.
        /// Usada somente quando Empenamento = Elastico.
        /// </summary>
        public double KOmega { get; set; }
    }
}
