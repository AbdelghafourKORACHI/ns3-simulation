#!/usr/bin/env python3
# plot_results.py - Analyse et visualisation des résultats QoS V14

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

# Style
plt.style.use('seaborn-v0_8-darkgrid')
sns.set_palette("husl")

def load_data(filename='results_V14.csv'):
    """Charge les données CSV"""
    try:
        df = pd.read_csv(filename)
        print(f"Data loaded: {len(df)} rows")
        return df
    except FileNotFoundError:
        print(f"File {filename} not found!")
        print("Please run ns-3 simulation first to generate results_V14.csv")
        return None

def plot_bar_charts(df):
    """Graphiques à barres comparatifs"""
    # Agrégation
    agg = df.groupby(['scenario', 'qos', 'type']).agg({
        'delay_ms': 'mean',
        'loss_pct': 'mean',
        'tput_mbps': 'mean',
        'mos': 'mean'
    }).reset_index()
    
    scenarios = agg['scenario'].unique()
    
    for scenario in scenarios:
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        axes = axes.flatten()
        sc_df = agg[agg['scenario'] == scenario]
        
        metrics = [('delay_ms', 'Delay (ms)', 150), 
                   ('loss_pct', 'Loss (%)', 1),
                   ('tput_mbps', 'Throughput (Mbps)', None),
                   ('mos', 'MOS', 3.5)]
        
        for idx, (metric, label, threshold) in enumerate(metrics):
            ax = axes[idx]
            types = sc_df['type'].unique()
            x = np.arange(len(types))
            width = 0.35
            
            qos_vals = sc_df[sc_df['qos'] == 1]
            noqos_vals = sc_df[sc_df['qos'] == 0]
            
            bars1 = ax.bar(x - width/2, 
                          [noqos_vals[noqos_vals['type'] == t][metric].values[0] if len(noqos_vals[noqos_vals['type'] == t]) > 0 else 0 for t in types],
                          width, label='Without QoS', color='#FF6B6B')
            
            bars2 = ax.bar(x + width/2,
                          [qos_vals[qos_vals['type'] == t][metric].values[0] if len(qos_vals[qos_vals['type'] == t]) > 0 else 0 for t in types],
                          width, label='With QoS', color='#4ECDC4')
            
            if threshold:
                ax.axhline(y=threshold, color='red', linestyle='--', alpha=0.5)
            
            ax.set_xlabel('Flow type')
            ax.set_ylabel(label)
            ax.set_title(f'{scenario} - {label}')
            ax.set_xticks(x)
            ax.set_xticklabels(types)
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        plt.suptitle(f'QoS Impact Comparison - {scenario}', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(f'bar_chart_{scenario}.png', dpi=150, bbox_inches='tight')
        plt.show()

def plot_mos_analysis(df):
    """Analyse MOS pour VoIP"""
    voip = df[df['type'] == 'VoIP'].dropna(subset=['mos', 'delay_ms', 'loss_pct'])
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    colors = {'SC1_WiFiStatic': 'blue', 'SC2_WiFiMobile': 'green', 'SC3_LTE': 'red'}
    markers = {0: 'o', 1: 's'}
    
    # MOS vs Delay
    for scenario in colors.keys():
        for qos in [0, 1]:
            subset = voip[(voip['scenario'] == scenario) & (voip['qos'] == qos)]
            if len(subset) > 0:
                ax1.scatter(subset['delay_ms'], subset['mos'],
                           label=f'{scenario} {"QoS" if qos else "NoQoS"}',
                           color=colors[scenario], marker=markers[qos], s=100, alpha=0.7)
    
    ax1.set_xlabel('Delay (ms)')
    ax1.set_ylabel('MOS')
    ax1.set_title('MOS vs Delay - VoIP')
    ax1.axhline(y=3.5, color='green', linestyle='--', alpha=0.5, label='Good (MOS>3.5)')
    ax1.axhline(y=2.5, color='orange', linestyle='--', alpha=0.5, label='Fair (2.5<MOS<3.5)')
    ax1.legend(loc='upper right', fontsize=9)
    ax1.grid(True, alpha=0.3)
    
    # MOS vs Loss
    for scenario in colors.keys():
        for qos in [0, 1]:
            subset = voip[(voip['scenario'] == scenario) & (voip['qos'] == qos)]
            if len(subset) > 0:
                ax2.scatter(subset['loss_pct'], subset['mos'],
                           label=f'{scenario} {"QoS" if qos else "NoQoS"}',
                           color=colors[scenario], marker=markers[qos], s=100, alpha=0.7)
    
    ax2.set_xlabel('Packet Loss (%)')
    ax2.set_ylabel('MOS')
    ax2.set_title('MOS vs Packet Loss - VoIP')
    ax2.axhline(y=3.5, color='green', linestyle='--', alpha=0.5)
    ax2.axhline(y=2.5, color='orange', linestyle='--', alpha=0.5)
    ax2.legend(loc='upper right', fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    plt.suptitle('VoIP Quality of Experience (MOS) Analysis', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('mos_analysis.png', dpi=150, bbox_inches='tight')
    plt.show()

def plot_heatmap(df):
    """Heatmap des performances"""
    agg = df.groupby(['scenario', 'qos', 'type']).agg({
        'delay_ms': 'mean',
        'loss_pct': 'mean'
    }).reset_index()
    
    # Score normalisé (plus petit = meilleur)
    for metric in ['delay_ms', 'loss_pct']:
        max_val = agg[metric].max()
        if max_val > 0:
            agg[f'{metric}_norm'] = 1 - (agg[metric] / max_val)
    
    agg['perf_score'] = (agg['delay_ms_norm'] + agg['loss_pct_norm']) / 2
    
    pivot = agg.pivot_table(index=['scenario', 'qos'], 
                            columns='type', 
                            values='perf_score')
    
    plt.figure(figsize=(10, 6))
    sns.heatmap(pivot, annot=True, cmap='RdYlGn', center=0.5,
                fmt='.3f', cbar_kws={'label': 'Performance Score'})
    plt.title('Performance Heatmap (Higher is Better)', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('performance_heatmap.png', dpi=150, bbox_inches='tight')
    plt.show()

if __name__ == "__main__":
    df = load_data()
    if df is not None:
        print("\n1. Generating bar charts...")
        plot_bar_charts(df)
        
        print("\n2. Generating MOS analysis...")
        plot_mos_analysis(df)
        
        print("\n3. Generating heatmap...")
        plot_heatmap(df)
        
        print("\nAll plots generated!")