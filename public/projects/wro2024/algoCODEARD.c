if (orange_y_coord > 50) {
    LEFT(90);
  } else if (orange_y_coord < -50) {
    RIGHT(90);
  } else if (orange_y_coord >= -50 && orange_y_coord <= 50 && orange_y_coord !=0) {
    FORWARDKICK();
  }