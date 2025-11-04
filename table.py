#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calculadora de Tabla de Amortización
====================================

Este script calcula y muestra una tabla de amortización para préstamos
con cuotas fijas utilizando el método francés.

Autor: Sistema PH2K
Fecha: 2024
"""

import math
from typing import List, Dict, Tuple


class CalculadoraAmortizacion:
    """Clase para calcular tablas de amortización de préstamos."""
    
    def __init__(self, principal: float, tasa_anual: float, plazo_meses: int):
        """
        Inicializa la calculadora con los parámetros del préstamo.
        
        Args:
            principal (float): Monto del préstamo
            tasa_anual (float): Tasa de interés anual (como decimal, ej: 0.12 para 12%)
            plazo_meses (int): Plazo del préstamo en meses
        """
        self.principal = principal
        self.tasa_anual = tasa_anual
        self.plazo_meses = plazo_meses
        self.tasa_mensual = tasa_anual / 12
        self.cuota_fija = self._calcular_cuota_fija()
    
    def _calcular_cuota_fija(self) -> float:
        """Calcula la cuota fija mensual usando la fórmula del método francés."""
        if self.tasa_mensual == 0:
            return self.principal / self.plazo_meses
        
        factor = (1 + self.tasa_mensual) ** self.plazo_meses
        return self.principal * (self.tasa_mensual * factor) / (factor - 1)
    
    def generar_tabla(self) -> List[Dict[str, float]]:
        """
        Genera la tabla de amortización completa.
        
        Returns:
            List[Dict]: Lista de diccionarios con los datos de cada período
        """
        tabla = []
        saldo_restante = self.principal
        
        for mes in range(1, self.plazo_meses + 1):
            interes = saldo_restante * self.tasa_mensual
            amortizacion = self.cuota_fija - interes
            saldo_restante = max(saldo_restante - amortizacion, 0)
            
            tabla.append({
                "mes": mes,
                "cuota": round(self.cuota_fija, 2),
                "interes": round(interes, 2),
                "amortizacion": round(amortizacion, 2),
                "saldo_restante": round(saldo_restante, 2)
            })
        
        return tabla
    
    def calcular_totales(self, tabla: List[Dict[str, float]]) -> Dict[str, float]:
        """Calcula los totales de la tabla de amortización."""
        total_cuotas = sum(fila["cuota"] for fila in tabla)
        total_intereses = sum(fila["interes"] for fila in tabla)
        total_amortizacion = sum(fila["amortizacion"] for fila in tabla)
        
        return {
            "total_cuotas": round(total_cuotas, 2),
            "total_intereses": round(total_intereses, 2),
            "total_amortizacion": round(total_amortizacion, 2)
        }


def imprimir_encabezado(principal: float, tasa_anual: float, plazo_meses: int, cuota_fija: float):
    """Imprime el encabezado con información del préstamo."""
    print("\n" + "="*80)
    print("🏦  TABLA DE AMORTIZACIÓN - MÉTODO FRANCÉS  🏦".center(80))
    print("="*80)
    print(f"💰 Monto del Préstamo:     ${principal:,.2f}")
    print(f"📊 Tasa Anual:             {tasa_anual*100:.2f}%")
    print(f"📅 Plazo:                  {plazo_meses} meses")
    print(f"💳 Cuota Mensual Fija:     ${cuota_fija:,.2f}")
    print("="*80)


def imprimir_tabla_formateada(tabla: List[Dict[str, float]]):
    """Imprime la tabla de amortización con formato visual mejorado."""
    # Encabezados de la tabla
    print(f"{'Mes':>4} │ {'Cuota':>10} │ {'Interés':>10} │ {'Amortización':>12} │ {'Saldo Restante':>15}")
    print("─"*4 + "┼" + "─"*11 + "┼" + "─"*11 + "┼" + "─"*13 + "┼" + "─"*16)
    
    # Filas de datos
    for fila in tabla:
        mes = fila["mes"]
        cuota = fila["cuota"]
        interes = fila["interes"]
        amortizacion = fila["amortizacion"]
        saldo = fila["saldo_restante"]
        
        # Colores para diferentes rangos (usando caracteres especiales)
        if saldo == 0:
            indicador = "✅"
        elif saldo < 5000:
            indicador = "🟡"
        else:
            indicador = "🔵"
        
        print(f"{mes:>4} │ ${cuota:>9.2f} │ ${interes:>9.2f} │ ${amortizacion:>11.2f} │ ${saldo:>14.2f} {indicador}")


def imprimir_resumen(totales: Dict[str, float]):
    """Imprime el resumen final con totales."""
    print("="*80)
    print("📋  RESUMEN FINANCIERO".center(80))
    print("="*80)
    print(f"💰 Total Pagado en Cuotas:     ${totales['total_cuotas']:,.2f}")
    print(f"📈 Total Pagado en Intereses:  ${totales['total_intereses']:,.2f}")
    print(f"🏠 Total Amortizado (Capital): ${totales['total_amortizacion']:,.2f}")
    print(f"💸 Costo Total del Préstamo:   ${totales['total_cuotas']:,.2f}")
    print(f"📊 Porcentaje de Intereses:    {(totales['total_intereses']/totales['total_cuotas']*100):.2f}%")
    print("="*80)


def main():
    """Función principal que ejecuta el cálculo y muestra la tabla."""
    # Parámetros del préstamo
    PRINCIPAL = 10000.00    # Monto del préstamo
    TASA_ANUAL = 0.12       # 12% anual
    PLAZO_MESES = 12        # 12 meses
    
    # Crear calculadora
    calculadora = CalculadoraAmortizacion(PRINCIPAL, TASA_ANUAL, PLAZO_MESES)
    
    # Generar tabla
    tabla = calculadora.generar_tabla()
    totales = calculadora.calcular_totales(tabla)
    
    # Mostrar resultados
    imprimir_encabezado(PRINCIPAL, TASA_ANUAL, PLAZO_MESES, calculadora.cuota_fija)
    imprimir_tabla_formateada(tabla)
    imprimir_resumen(totales)
    
    print("\n✨ Cálculo completado exitosamente ✨\n")


if __name__ == "__main__":
    main()
