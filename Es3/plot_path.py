import pandas as pd
import matplotlib.pyplot as plt
import math
import os

# Nome del file CSV salvato dal nodo C++
CSV_FILE = "vc_path_projected.csv"

def plot_vc_path():
    """
    Carica i dati del percorso dal CSV, calcola la distanza finale 
    e traccia il percorso, la posizione finale del VC e la CS.
    """
    if not os.path.exists(CSV_FILE):
        print(f"Errore: File '{CSV_FILE}' non trovato.")
        print("Assicurati che il nodo C++ ('path_tracker_listener') sia stato eseguito e terminato (Ctrl+C) per salvare il file.")
        return

    # Carica i dati
    df = pd.read_csv(CSV_FILE)

    # Verifica se ci sono dati validi
    if df.empty:
        print("Errore: Il file CSV è vuoto.")
        return

    # --- 1. Estrarre le Posizioni ---
    
    # Posizione finale del VC (ultima riga del DataFrame)
    final_vc_x = df['x'].iloc[-1]
    final_vc_y = df['y'].iloc[-1]

    # Posizione della CS (prendiamo la prima riga, dato che la CS è fissa rispetto al pavimento)
    cs_x = df['cs_x'].iloc[0]
    cs_y = df['cs_y'].iloc[0]

    # --- 2. Calcolo della Distanza Finale (Domanda: How far is it now?) ---
    distance = math.sqrt((final_vc_x - cs_x)**2 + (final_vc_y - cs_y)**2)
    
    # --- 3. Plotting ---
    plt.figure(figsize=(10, 8))
    
    # Traccia il percorso completo del VC (rispetto al pavimento tag36h11:0)
    plt.plot(df['x'], df['y'], 
             label='VC Path (Projected on tag36h11:0)', 
             color='blue', linewidth=1.5, alpha=0.7)
    
    # Evidenzia la posizione finale del VC (Robot)
    plt.scatter(final_vc_x, final_vc_y, 
                color='red', s=150, zorder=5, 
                label='VC Final Position', marker='o')
    
    # Evidenzia la posizione della CS (Charging Station)
    plt.scatter(cs_x, cs_y, 
                color='green', s=200, marker='s', zorder=5, 
                label='CS Position (tag36h11:1)')
    
    # Linea che collega la posizione finale del VC alla CS
    plt.plot([final_vc_x, cs_x], [final_vc_y, cs_y], 
             'k--', linewidth=1, alpha=0.6, 
             label=f'Final Distance: {distance:.3f} m')

    # Aggiungi etichette e titolo
    plt.title(f'Percorso del Robot (VC) rispetto al Pavimento (tag36h11:0)')
    plt.xlabel('X (metri)')
    plt.ylabel('Y (metri)')
    plt.legend(loc='best')
    plt.axis('equal') # Assicura che la scala X e Y sia la stessa
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    # Assicurati di avere installato pandas e matplotlib:
    # pip install pandas matplotlib
    plot_vc_path()