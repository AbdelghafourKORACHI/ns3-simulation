#!/usr/bin/env python3
# statistical_analysis.py - Tests statistiques

import pandas as pd
import numpy as np
from scipy import stats
import matplotlib.pyplot as plt

def load_data(filename='results_V14.csv'):
    try:
        df = pd.read_csv(filename)
        return df
    except FileNotFoundError:
        print(f"File {filename} not found!")
        return None

def compute_improvements(df):
    """Calcule les améliorations apportées par la QoS"""
    results = []
    
    for scenario in df['scenario'].unique():
        for ftype in ['VoIP', 'Video', 'FTP']:
            subset = df[(df['scenario'] == scenario) & (df['type'] == ftype)]
            
            if len(subset) > 0:
                # Délai
                delay_qos = subset[subset['qos'] == 1]['delay_ms'].dropna()
                delay_noqos = subset[subset['qos'] == 0]['delay_ms'].dropna()
                
                if len(delay_qos) > 0 and len(delay_noqos) > 0:
                    delay_imp = ((delay_noqos.mean() - delay_qos.mean()) / delay_noqos.mean()) * 100
                    t_stat, p_val = stats.ttest_ind(delay_qos, delay_noqos)
                    
                    results.append({
                        'Scenario': scenario,
                        'Flow': ftype,
                        'Metric': 'Delay',
                        'Without_QoS': f"{delay_noqos.mean():.1f}ms",
                        'With_QoS': f"{delay_qos.mean():.1f}ms",
                        'Improvement': f"{delay_imp:.1f}%",
                        'p-value': f"{p_val:.4f}",
                        'Significant': 'Yes' if p_val < 0.05 else 'No'
                    })
                
                # Perte
                loss_qos = subset[subset['qos'] == 1]['loss_pct'].dropna()
                loss_noqos = subset[subset['qos'] == 0]['loss_pct'].dropna()
                
                if len(loss_qos) > 0 and len(loss_noqos) > 0:
                    loss_imp = ((loss_noqos.mean() - loss_qos.mean()) / max(loss_noqos.mean(), 0.01)) * 100
                    t_stat, p_val = stats.ttest_ind(loss_qos, loss_noqos)
                    
                    results.append({
                        'Scenario': scenario,
                        'Flow': ftype,
                        'Metric': 'Loss',
                        'Without_QoS': f"{loss_noqos.mean():.1f}%",
                        'With_QoS': f"{loss_qos.mean():.1f}%",
                        'Improvement': f"{loss_imp:.1f}%",
                        'p-value': f"{p_val:.4f}",
                        'Significant': 'Yes' if p_val < 0.05 else 'No'
                    })
    
    return pd.DataFrame(results)

def main():
    df = load_data()
    if df is None:
        return
    
    print("\n" + "="*70)
    print("STATISTICAL ANALYSIS - QoS IMPROVEMENTS")
    print("="*70)
    
    results = compute_improvements(df)
    print("\n", results.to_string(index=False))
    
    # Sauvegarde
    results.to_csv('statistical_results.csv', index=False)
    print("\nResults saved to 'statistical_results.csv'")
    
    # Graphique des améliorations
    fig, ax = plt.subplots(figsize=(12, 6))
    
    delay_results = results[results['Metric'] == 'Delay']
    x = np.arange(len(delay_results))
    width = 0.35
    
    improvements = [float(r.split('%')[0]) for r in delay_results['Improvement']]
    colors = ['green' if imp > 50 else 'orange' if imp > 20 else 'red' for imp in improvements]
    
    bars = ax.bar(x, improvements, width, color=colors)
    
    # Ajout des valeurs
    for bar, imp in zip(bars, improvements):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
               f'{imp:.1f}%', ha='center', va='bottom', fontsize=9)
    
    ax.set_ylabel('Delay Improvement (%)')
    ax.set_title('QoS Impact on Delay Reduction')
    ax.set_xticks(x)
    ax.set_xticklabels([f"{row['Scenario']}\n{row['Flow']}" for _, row in delay_results.iterrows()])
    ax.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('statistical_improvements.png', dpi=150, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    main()